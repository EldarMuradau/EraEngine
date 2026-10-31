#include "engine/engine.h"

#include "core/input.h"
#include "core/imgui.h"
#include "core/cpu_profiling.h"
#include "core/job_system.h"
#include "core/frame_params.h"
#include "core/debug/debug_var_storage.h"

#include "dx/dx_context.h"
#include "dx/dx_command_list.h"
#include "dx/dx_profiling.h"

#include "window/dx_window.h"

#include "asset/file_registry.h"

#include "editor/file_browser.h"
#include "editor/asset_editor_panel.h"
#include "editor/transformation_gizmo.h"

#include "ecs/world_system_scheduler.h"

#include "core/editor_icons.h"
#include "core/log.h"

#include "rendering/render_utils.h"
#include "rendering/main_renderer.h"

#include "audio/audio.h"

#include <clara/clapa.hpp>

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui/imgui_internal.h>

#include <fstream>

#include <timeapi.h>

#pragma comment(lib, "winmm.lib")

namespace era_engine
{
    bool handle_windows_messages();

    static FrameParams frames[NUM_BUFFERED_FRAMES];
    static uint64 frame_id;

    // Execution of the last collected frame. Executions run one at a time, in frame order.
    static JobCounter frame_execution;

    static bool serial_frames = false;

    void wait_for_frame_execution()
    {
        ZoneScopedN("Wait for frame execution");
        JobSystem::wait_for_counter(&frame_execution);
    }

    // Starts collection of the next frame once the GPU has finished the frame that used the same slot.
    static FrameParams& begin_frame_params()
    {
        FrameParams& frame = frames[frame_id % NUM_BUFFERED_FRAMES];

        {
            ZoneScopedN("Wait for queued frame to finish rendering");
            dx_context.render_queue.wait_for_fence(frame.gpu_fence);
        }

        frame.frame_id = frame_id;
        frame.slot = (uint32)(frame_id % NUM_BUFFERED_FRAMES);
        frame.gpu_fence = 0;
        frame.render_tasks.clear();

        JobSystem::set_frame_context(&frame);

        {
            ZoneScopedN("dx_context.new_frame");
            dx_context.new_frame(frame_id);
        }

        return frame;
    }

    static bool new_frame(float& dt)
    {
        static bool first = true;
        static float perf_freq;
        static LARGE_INTEGER last_time;

        if (first)
        {
            LARGE_INTEGER perf_freq_result;
            QueryPerformanceFrequency(&perf_freq_result);
            perf_freq = (float)perf_freq_result.QuadPart;

            QueryPerformanceCounter(&last_time);

            first = false;
        }

        LARGE_INTEGER current_time;
        QueryPerformanceCounter(&current_time);
        dt = ((float)(current_time.QuadPart - last_time.QuadPart) / perf_freq);
        last_time = current_time;

        bool result = handle_windows_messages();

        new_imgui_frame(dt);
        ImGui::DockSpaceOverViewport(nullptr, ImGuiDockNodeFlags_PassthruCentralNode);

        cpu_profiling_resolve_time_stamps();

        return result;
    }

    static void draw_debug_menu_bar(float elapsed, uint64 gameplay_fixed_frame_id)
    {
        const float fps = 1.0f / elapsed;
        bool debug_vars_opened = false;
        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu(std::to_string(fps).c_str()))
            {
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu(std::to_string(frame_id).c_str()))
            {
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu(std::to_string(gameplay_fixed_frame_id).c_str()))
            {
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Debug Vars Menu"))
            {
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        DebugVarsStorage::get_instance()->draw_vars(debug_vars_opened);
    }

    static void render_to_main_window(DXWindow& window, FrameParams& frame, const vec4& clear_color)
    {
        ZoneScopedN("Engine::render_to_main_window");

        DXResource backbuffer = window.back_buffers[window.current_backbuffer_index];
        DXRTVDescriptorHandle rtv = window.back_buffer_rtvs[window.current_backbuffer_index];

        DXCommandList* cl = dx_context.get_free_render_command_list();

        {
            DX_PROFILE_BLOCK(cl, "Blit to backbuffer");

            CD3DX12_VIEWPORT viewport = CD3DX12_VIEWPORT(0.f, 0.f, (float)window.client_width, (float)window.client_height);
            cl->set_viewport(viewport);

            const ref<DXTexture>& frame = window.present_texture;
            const bool copy_frame = frame != nullptr && frame->width == window.client_width && frame->height == window.client_height && frame->format == color_depth_to_format(window.color_depth);

            if (copy_frame)
            {
                cl->transition_barrier(frame, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_SOURCE);
                cl->transition_barrier(backbuffer, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_DEST);
                cl->copy_resource(frame->resource, backbuffer);
                cl->transition_barrier(frame, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_COMMON);
                cl->transition_barrier(backbuffer, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_RENDER_TARGET);
            }
            else
            {
                cl->transition_barrier(backbuffer, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_RENDER_TARGET);
                cl->clear_rtv(rtv, clear_color.x, clear_color.y, clear_color.z);
            }
            window.present_texture = nullptr;

            cl->set_render_target(&rtv, 1, 0);

            render_imgui(cl);

            cl->transition_barrier(backbuffer, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
        }
        dx_context.end_frame(cl);

        uint64 result = dx_context.execute_command_list(cl);

        window.swap_buffers();

        frame.gpu_fence = result;
    }

    // Execution of a collected frame: queued render tasks, then the main window.
    static void execute_frame(DXWindow& window, FrameParams& frame, const vec4& clear_color)
    {
        ZoneScopedN("Engine::execute_frame");

        {
            ZoneScopedN("Frame render tasks");
            for (const std::function<void()>& task : frame.render_tasks)
            {
                task();
            }
        }

        render_to_main_window(window, frame, clear_color);
    }

    struct FrameExecution
    {
        DXWindow* window = nullptr;
        FrameParams* frame = nullptr;
        vec4 clear_color;
    };

    static FrameExecution frame_executions[NUM_BUFFERED_FRAMES];

    // Runs the frame's execution as a job while the main thread starts the next frame.
    static void launch_frame_execution(DXWindow& window, FrameParams& frame, const vec4& clear_color)
    {
        wait_for_frame_execution();

        FrameExecution& execution = frame_executions[frame.slot];
        execution = FrameExecution{&window, &frame, clear_color};

        const JobDecl decl([](void* data)
            {
                const FrameExecution& execution = *static_cast<const FrameExecution*>(data);
                execute_frame(*execution.window, *execution.frame, execution.clear_color);
            },
            &execution, JobPriority::HIGH, false, "Frame execution");
        JobSystem::run_jobs(&decl, 1, &frame_execution);

        if (serial_frames)
        {
            wait_for_frame_execution();
        }
    }

    Engine* Engine::instance_object = nullptr;

    static bool verbose = false;
    static bool main_menu = false;

    static MMRESULT timer_result;

    Engine::Engine(int argc, char** argv)
    {
        using namespace clara;

        running = true;
        instance_object = this;

#ifndef _DEBUG
        ::ShowWindow(::GetConsoleWindow(), SW_HIDE);
#endif

        Parser cli;
        cli += Opt(verbose, "verbose")["-v"]["--verbose"]("Enable verbose logging");
        cli += Opt(main_menu, "main-menu")["-mm"]["--main-menu"]("Enable main menu bar");
        cli += Opt(serial_frames, "serial-frames")["-sf"]["--serial-frames"]("Execute each frame before collecting the next one");

        auto result = cli.parse(Args(argc, argv));
        if (!result)
        {
            std::cerr << "Error in command line: " << result.errorMessage() << std::endl;
        }

        if (verbose)
        {
            ::ShowWindow(::GetConsoleWindow(), SW_SHOW);
        }

        const bool dx_inited = dx_context.initialize();
        ASSERT(dx_inited);

        timer_result = timeBeginPeriod(1);
        if (timer_result != TIMERR_NOERROR)
        {
            std::cerr << "Failed to set precise Windows timer resolution." << std::endl;
        }

        std::cout << "Windows scheduler precision set to 1ms. Running physics thread..." << std::endl;
    }

    Engine::~Engine()
    {
    }

    Engine* Engine::instance()
    {
        return instance_object;
    }

    void Engine::release()
    {
        instance()->running = false;
    }

    Engine* Engine::create_instance(int argc, char** argv)
    {
        return new Engine(argc, argv);
    }

    bool Engine::run(const std::function<void(void)>& initial_task /* = nullptr */)
    {
        initialize_job_system();
        initialize_file_registry();

        initialize_render_utils();

        DXWindow* window = get_object<DXWindow>();
        initialize_imgui(*window);

        FileBrowser file_browser;

        // Wait for initialization to finish
        dx_context.flush_application();

        if (initial_task)
        {
            initial_task();
        }

        execute_main_thread_jobs();

        for (auto& [name, world] : get_worlds())
        {
            WorldSystemScheduler* scheduler = world->get_system_scheduler();
            scheduler->initialize_all_systems();
        }

        float dt = 0.0f;
        bool status = true;
        while (status)
        {
            ZoneScopedN("Engine::update");

            window->begin_frame(dt);

            FrameParams& frame = begin_frame_params();

            {
                ZoneScopedN("Engine::start_frame");
                status = new_frame(dt);
            }
            frame.dt = dt;

            window->update_input(dt);

            uint64 gameplay_fixed_frame_id = 0;

            {
                ZoneScopedN("Engine::update_worlds");

                auto& worlds = get_worlds();
                for (auto& [name, world] : worlds)
                {
                    ZoneScopedN("Engine::normal_update_world");

                    WorldSystemScheduler* scheduler = world->get_system_scheduler();
                    scheduler->update_normal(dt);

                    if (name == World::GAMEPLAY_WORLD_NAME && gameplay_fixed_frame_id == 0)
                    {
                        gameplay_fixed_frame_id = world->get_fixed_frame_id();
                    }
                }
            }

            if (main_menu)
            {
                draw_debug_menu_bar(dt, gameplay_fixed_frame_id);
            }

            {
                ZoneScopedN("Engine::execute_main_thread_jobs");

                execute_main_thread_jobs();
            }

            {
                ZoneScopedN("Engine::render");
                file_browser.draw();

                const ImVec4 clear_color = ImGui::GetStyleColorVec4(ImGuiCol_DockingEmptyBg);
                end_imgui_frame();

                launch_frame_execution(*window, frame, vec4(clear_color.x, clear_color.y, clear_color.z, clear_color.w));
            }

            cpu_profiling_frame_end_marker();

            ++frame_id;

            FrameMark;
        }

        wait_for_frame_execution();

        for (auto& [name, world] : get_worlds())
        {
            WorldSystemScheduler* scheduler = world->get_system_scheduler();
            scheduler->stop();
        }

        shutdown_job_system();

        terminate();

        if (timer_result == TIMERR_NOERROR)
        {
            timeEndPeriod(1);
        }

        return true;
    }

    void Engine::terminate()
    {
        dx_context.flush_application();

        dx_context.quit();

        instance_object = nullptr;
    }

    bool Engine::update()
    {
        return true;
    }

}