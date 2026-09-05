#include "engine/engine.h"

#include "core/input.h"
#include "core/imgui.h"
#include "core/cpu_profiling.h"
#include "core/job_system.h"
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
	bool handleWindowsMessages();

	static uint64 fenceValues[NUM_BUFFERED_FRAMES];
	static uint64 frameID;

	static bool newFrame(float& dt, dx_window& window)
	{
		static bool first = true;
		static float perfFreq;
		static LARGE_INTEGER lastTime;

		if (first)
		{
			LARGE_INTEGER perfFreqResult;
			QueryPerformanceFrequency(&perfFreqResult);
			perfFreq = (float)perfFreqResult.QuadPart;

			QueryPerformanceCounter(&lastTime);

			first = false;
		}

		LARGE_INTEGER currentTime;
		QueryPerformanceCounter(&currentTime);
		dt = ((float)(currentTime.QuadPart - lastTime.QuadPart) / perfFreq);
		lastTime = currentTime;

		bool result = handleWindowsMessages();

		newImGuiFrame(dt);
		ImGui::DockSpaceOverViewport();

		cpu_profiling_resolve_time_stamps();

		{
			ZoneScopedN("Wait for queued frame to finish rendering");
			dxContext.renderQueue.waitForFence(fenceValues[window.currentBackbufferIndex]);
		}

		{
			ZoneScopedN("dxContext.newFrame");
			dxContext.newFrame(frameID);
		}

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
			if (ImGui::BeginMenu(std::to_string(frameID).c_str()))
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

	static void renderToMainWindow(dx_window& window)
	{
		dx_resource backbuffer = window.backBuffers[window.currentBackbufferIndex];
		dx_rtv_descriptor_handle rtv = window.backBufferRTVs[window.currentBackbufferIndex];

		dx_command_list* cl = dxContext.getFreeRenderCommandList();

		{
			DX_PROFILE_BLOCK(cl, "Blit to backbuffer");

			CD3DX12_VIEWPORT viewport = CD3DX12_VIEWPORT(0.f, 0.f, (float)window.clientWidth, (float)window.clientHeight);
			cl->setViewport(viewport);

			cl->transitionBarrier(backbuffer, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_RENDER_TARGET);

			cl->clearRTV(rtv, 0.f, 0.f, 0.f);
			cl->setRenderTarget(&rtv, 1, 0);

			renderImGui(cl);

			cl->transitionBarrier(backbuffer, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
		}
		dxContext.endFrame(cl);

		uint64 result = dxContext.executeCommandList(cl);

		window.swapBuffers();

		fenceValues[window.currentBackbufferIndex] = result;
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

		auto result = cli.parse(Args(argc, argv));
		if (!result)
		{
			std::cerr << "Error in command line: " << result.errorMessage() << std::endl;
		}

		if (verbose)
		{
			::ShowWindow(::GetConsoleWindow(), SW_SHOW);
		}

		const bool dx_inited = dxContext.initialize();
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

		initializeRenderUtils();

		dx_window* window = get_object<dx_window>();
		initializeImGui(*window);

		file_browser fileBrowser;

		// Wait for initialization to finish
		fenceValues[NUM_BUFFERED_FRAMES - 1] = dxContext.renderQueue.signal();
		dxContext.flushApplication();

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

			{
				ZoneScopedN("Engine::start_frame");
				status = newFrame(dt, *window);
			}

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
				fileBrowser.draw();

				ImGui::End();

				renderToMainWindow(*window);
			}

			cpu_profiling_frame_end_marker();

			++frameID;

			FrameMark;
		}

	for (auto& [name, world] : get_worlds())
	{
		WorldSystemScheduler* scheduler = world->get_system_scheduler();
		scheduler->stop();
	}

	terminate();

	if (timer_result == TIMERR_NOERROR)
	{
		timeEndPeriod(1);
	}

	return true;
}

void Engine::terminate()
{
	dxContext.flushApplication();

	dxContext.quit();

	instance_object = nullptr;
}

bool Engine::update()
{
	return true;
}

}