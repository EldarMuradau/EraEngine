#include "rendering/ecs/private/render_system.h"
#include "rendering/ecs/renderer_holder_root_component.h"

#include "core/imgui.h"
#include "core/editor_icons.h"
#include "core/log.h"
#include "core/string.h"
#include "core/cpu_profiling.h"
#include "core/ecs/private/input_system.h"
#include "core/ecs/input_receiver_component.h"

#include "engine/engine.h"

#include "core/frame_params.h"
#include "core/jobs/jobs.h"

#include "window/dx_window.h"

#include "rendering/mesh_shader.h"
#include "rendering/pbr_material.h"
#include "rendering/render_utils.h"

#include "ecs/update_groups.h"
#include "ecs/base_components/transform_component.h"
#include "ecs/rendering/world_renderer.h"
#include "ecs/rendering/scene_rendering.h"
#include "core/ecs/camera_holder_component.h"
#include "ecs/rendering/mesh_component.h"
#include "ecs/prefab/prefab_utils.h"

#include "audio/audio.h"

#include "terrain/terrain.h"

#include "animation/skinning.h"

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
    RTTR_REGISTRATION
    {
        using namespace rttr;

        registration::class_<RenderSystem>("RenderSystem")
            .constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("render")))
            .method("begin_frame", &RenderSystem::begin_frame)(metadata("update_group", update_types::INPUT),
                metadata("Before", std::vector<std::string>{"InputSystem::update"}))
            .method("before_render", &RenderSystem::before_render)(metadata("update_group", update_types::BEFORE_RENDER))
            .method("update", &RenderSystem::update)(metadata("update_group", update_types::RENDER))
            .method("after_render", &RenderSystem::after_render)(metadata("update_group", update_types::AFTER_RENDER));
    }

    RenderSystem::RenderSystem(World* _world)
        : System(_world)
    {
        load_custom_shaders();

        renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();
        ASSERT(renderer_holder_rc != nullptr);

        if (dx_context.feature_support.raytracing())
        {
            for (FramePasses& frame : passes)
            {
                frame.raytracing_tlas.initialize();
            }
        }

        renderer_holder_rc->camera.initialize_ingame(vec3::zero, quat::identity, deg2rad(70.f), 0.2f);
        renderer_holder_rc->environment.set_from_texture(get_asset_path("/resources/assets/sky/sunset_in_the_chalk_quarry_4k.hdr"));
        renderer_holder_rc->environment.light_probe_grid.initialize(vec3(-20.f, -1.f, -20.f), vec3(40.f, 20.f, 40.f), 1.5f);

        renderer_holder_rc->sun.direction = normalize(vec3(-0.6f, -1.f, -0.3f));
        renderer_holder_rc->sun.color = vec3(1.f, 0.93f, 0.76f);
        renderer_holder_rc->sun.intensity = 11.1f;
        renderer_holder_rc->sun.num_shadow_cascades = 3;
        renderer_holder_rc->sun.shadow_dimensions = 2048;
        renderer_holder_rc->sun.cascade_distances = vec4(9.f, 25.f, 50.f, 10000.f);
        renderer_holder_rc->sun.bias = vec4(0.02f, 0.03f, 0.05f, 0.1f);
        renderer_holder_rc->sun.blend_distances = vec4(5.f, 10.f, 10.f, 10.f);
        renderer_holder_rc->sun.stabilize = true;

        for (uint32 i = 0; i < NUM_BUFFERED_FRAMES; ++i)
        {
            point_light_buffer[i] = create_upload_buffer(sizeof(PointLightCB), 512, 0);
            spot_light_buffer[i] = create_upload_buffer(sizeof(SpotLightCB), 512, 0);
            decal_buffer[i] = create_upload_buffer(sizeof(PBRDecalCB), 512, 0);

            spot_light_shadow_info_buffer[i] = create_upload_buffer(sizeof(SpotShadowInfo), 512, 0);
            point_light_shadow_info_buffer[i] = create_upload_buffer(sizeof(PointShadowInfo), 512, 0);

            SET_NAME(point_light_buffer[i]->resource, "Point lights");
            SET_NAME(spot_light_buffer[i]->resource, "Spot lights");
            SET_NAME(decal_buffer[i]->resource, "Decals");

            SET_NAME(spot_light_shadow_info_buffer[i]->resource, "Spot light shadow infos");
            SET_NAME(point_light_shadow_info_buffer[i]->resource, "Point light shadow infos");
        }

        stack_arena.initialize();
    }

    RenderSystem::~RenderSystem()
    {
    }

    void RenderSystem::init()
    {
    }

    void RenderSystem::begin_frame(float dt)
    {
        ZoneScopedN("RenderSystem::begin_frame");

        // Debug drawing of this frame goes to its own slot.
        renderer_holder_rc->ldr_render_pass = &frame_passes().ldr_render_pass;

        if (renderer_holder_rc->present_mode == RendererHolderRootComponent::PRESENT_TO_IMGUI_WINDOW)
        {
            draw_imgui_viewport();
            return;
        }

        const DXWindow* window = get_object<DXWindow>();
        if (window->client_width > 0 && window->client_height > 0)
        {
            renderer_holder_rc->width = window->client_width;
            renderer_holder_rc->height = window->client_height;
        }

        const ImGuiPayload* payload = ImGui::GetDragDropPayload();
        if (payload != nullptr && payload->IsDataType(EDITOR_ICON_PREFAB) && ImGui::IsMouseReleased(ImGuiMouseButton_Left) && !ImGui::GetIO().WantCaptureMouse)
        {
            instantiate_dropped_prefab(static_cast<const char*>(payload->Data));
        }
    }

    void RenderSystem::before_render(float dt)
    {
        ZoneScopedN("RenderSystem::before_render");
        MainRenderer* renderer = renderer_holder_rc->renderer;

        renderer->begin_frame(renderer_holder_rc->width, renderer_holder_rc->height);

        reset_render_passes();
        frame_passes().compute_pass.dt = dt;

        stack_arena.reset();
    }

    void RenderSystem::update(float dt)
    {
        ZoneScopedN("RenderSystem::update");
        MainRenderer* renderer = renderer_holder_rc->renderer;

        DirectionalLight& sun = renderer_holder_rc->sun;
        PBREnvironment& environment = renderer_holder_rc->environment;

        RenderCamera& camera = renderer_holder_rc->camera;

        environment.update(sun.direction);
        sun.update_matrices(camera);
        set_audio_listener(camera.position, camera.rotation, vec3(0.f));
        environment.light_probe_grid.visualize(&frame_passes().opaque_render_pass);

        {
            for (auto [entity_handle, transform, terrain] : world->group(components_group<TransformComponent, TerrainComponent>).each())
            {
                terrain.update();
            }

            SceneLighting lighting;
            lighting.spot_light_buffer = spot_light_buffer[dx_context.get_buffered_frame_id()];
            lighting.point_light_buffer = point_light_buffer[dx_context.get_buffered_frame_id()];
            lighting.spot_light_shadow_info_buffer = spot_light_shadow_info_buffer[dx_context.get_buffered_frame_id()];
            lighting.point_light_shadow_info_buffer = point_light_shadow_info_buffer[dx_context.get_buffered_frame_id()];
            lighting.spot_shadow_render_passes = frame_passes().spot_shadow_render_passes;
            lighting.point_shadow_render_passes = frame_passes().point_shadow_render_passes;
            lighting.max_num_spot_shadow_render_passes = arraysize(frame_passes().spot_shadow_render_passes);
            lighting.max_num_point_shadow_render_passes = arraysize(frame_passes().point_shadow_render_passes);

            render_world(camera, world, stack_arena, Entity::null_handle, sun, lighting, !renderer->settings.cache_shadow_map,
                &frame_passes().opaque_render_pass, &frame_passes().transparent_render_pass, renderer_holder_rc->ldr_render_pass, &frame_passes().sun_shadow_render_pass, &frame_passes().compute_pass, dt);

            renderer->set_spot_lights(spot_light_buffer[dx_context.get_buffered_frame_id()], world->number_of_components_of_type<SpotLightComponent>(), spot_light_shadow_info_buffer[dx_context.get_buffered_frame_id()]);
            renderer->set_point_lights(point_light_buffer[dx_context.get_buffered_frame_id()], world->number_of_components_of_type<PointLightComponent>(), point_light_shadow_info_buffer[dx_context.get_buffered_frame_id()]);

            if (decals.size())
            {
                update_upload_buffer_data(decal_buffer[dx_context.get_buffered_frame_id()], decals.data(), (uint32)(sizeof(PBRDecalCB) * decals.size()));
                renderer->set_decals(decal_buffer[dx_context.get_buffered_frame_id()], (uint32)decals.size(), decal_texture);
            }

            submit_renderer_params(lighting.num_spot_shadow_render_passes, lighting.num_point_shadow_render_passes);
        }
    }

    void RenderSystem::after_render(float dt)
    {
        ZoneScopedN("RenderSystem::after_render");

        MainRenderer* renderer = renderer_holder_rc->renderer;

        DirectionalLight& sun = renderer_holder_rc->sun;
        PBREnvironment& environment = renderer_holder_rc->environment;

        RenderCamera& camera = renderer_holder_rc->camera;

        animation::perform_skinning(&frame_passes().compute_pass);

        if (dx_context.feature_support.raytracing())
        {
            build_raytracing_scene();
            renderer->set_raytracing_scene(&frame_passes().raytracing_tlas);
        }

        renderer->set_environment(environment);
        renderer->set_sun(sun);
        renderer->set_camera(camera);

        renderer->end_collection();

        // Recording and submission belong to the frame's execution.
        const bool present_to_main_window = renderer_holder_rc->present_mode == RendererHolderRootComponent::PRESENT_TO_MAIN_WINDOW;
        auto execute = [renderer, input = get_current_frame_input(), present_to_main_window]()
        {
            {
                ZoneScopedN("end_frame_common");
                end_frame_common();
            }

            renderer->end_frame(&input);

            if (present_to_main_window)
            {
                get_object<DXWindow>()->present_texture = renderer->frame_result;
            }
        };

        if (FrameParams* frame = JobSystem::get_frame_context())
        {
            frame->render_tasks.push_back(std::move(execute));
        }
        else
        {
            execute();
        }
    }

    RenderSystem::FramePasses& RenderSystem::frame_passes()
    {
        return passes[dx_context.get_buffered_frame_id()];
    }

    void RenderSystem::draw_imgui_viewport()
    {
        ImGui::BeginWindowHiddenTabBar("Scene Viewport");
        ImVec2 resolution = ImGui::GetContentRegionAvail();

        if (resolution.x > 0 && resolution.y > 0)
        {
            renderer_holder_rc->width = (uint32)resolution.x;
            renderer_holder_rc->height = (uint32)resolution.y;
        }

        MainRenderer* renderer = renderer_holder_rc->renderer;

        ImGui::Image(renderer->frame_result, renderer_holder_rc->width, renderer_holder_rc->height);

        if (ImGui::BeginDragDropTarget())
        {
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(EDITOR_ICON_PREFAB))
            {
                instantiate_dropped_prefab(static_cast<const char*>(payload->Data));
            }
            ImGui::EndDragDropTarget();
        }

        ImGui::End();
    }

    void RenderSystem::instantiate_dropped_prefab(const char* path)
    {
        const RenderCamera& camera = renderer_holder_rc->camera;

        PrefabInstantiateParams params;
        params.transform = trs(camera.position + camera.rotation * vec3(0.0f, 0.0f, -3.0f));
        PrefabUtils::instantiate_async(world, fs::path(path), params);
    }

    static bool is_texture_loading(const ref<DXTexture>& texture)
    {
        if (!texture)
        {
            return false;
        }
        const AssetLoadState state = texture->load_state.load(std::memory_order_acquire);
        return state == AssetLoadState::UNLOADED || state == AssetLoadState::LOADING;
    }

    static bool is_mesh_ready_for_raytracing(const MultiMesh& mesh)
    {
        if (mesh.load_state.load(std::memory_order_acquire) != AssetLoadState::LOADED || mesh.submeshes.empty())
        {
            return false;
        }

        for (const Submesh& submesh : mesh.submeshes)
        {
            const ref<PBRMaterial>& material = submesh.material;
            if (material && (is_texture_loading(material->albedo) || is_texture_loading(material->normal) || is_texture_loading(material->roughness) || is_texture_loading(material->metallic)))
            {
                return false;
            }
        }
        return true;
    }

    const RaytracingObjectType* RenderSystem::get_raytracing_object_type(const ref<MultiMesh>& mesh)
    {
        auto it = raytracing_mesh_types.find(mesh.get());
        if (it == raytracing_mesh_types.end() || it->second.mesh.expired())
        {
            if (!is_mesh_ready_for_raytracing(*mesh))
            {
                return nullptr;
            }
            it = raytracing_mesh_types.insert_or_assign(mesh.get(), RaytracingMeshEntry{mesh, define_blas_from_mesh(mesh)}).first;
        }

        return it->second.type.blas ? &it->second.type : nullptr;
    }

    void RenderSystem::build_raytracing_scene()
    {
        frame_passes().raytracing_tlas.reset();

        const MainRenderer* renderer = renderer_holder_rc->renderer;
        if (renderer->mode != RENDERER_MODE_PATHTRACED && renderer_holder_rc->environment.gi_mode != ENVIRONMENT_GI_RAYTRACED)
        {
            return;
        }

        for (auto [entity_handle, transform, mesh] : world->group(components_group<TransformComponent, MeshComponent>).each())
        {
            if (!mesh.mesh || mesh.is_hidden)
            {
                continue;
            }

            if (const RaytracingObjectType* type = get_raytracing_object_type(mesh.mesh))
            {
                frame_passes().raytracing_tlas.instantiate(*type, transform.get_world_transform());
            }
        }

        for (auto [entity_handle, transform, raytrace] : world->group(components_group<TransformComponent, RaytraceComponent>).each())
        {
            if (raytrace.type.blas)
            {
                frame_passes().raytracing_tlas.instantiate(raytrace.type, transform.get_world_transform());
            }
        }
    }

    void RenderSystem::load_custom_shaders()
    {
        if (dx_context.feature_support.mesh_shaders())
        {
            initialize_mesh_shader();
        }
    }

    void RenderSystem::reset_render_passes()
    {
        frame_passes().opaque_render_pass.reset();
        frame_passes().transparent_render_pass.reset();
        renderer_holder_rc->ldr_render_pass->reset();
        frame_passes().sun_shadow_render_pass.reset();
        frame_passes().compute_pass.reset();

        for (uint32 i = 0; i < arraysize(frame_passes().spot_shadow_render_passes); ++i)
        {
            frame_passes().spot_shadow_render_passes[i].reset();
        }

        for (uint32 i = 0; i < arraysize(frame_passes().point_shadow_render_passes); ++i)
        {
            frame_passes().point_shadow_render_passes[i].reset();
        }
    }

    void RenderSystem::submit_renderer_params(uint32 num_spot_light_shadow_passes, uint32 num_point_light_shadow_passes)
    {
        MainRenderer* renderer = renderer_holder_rc->renderer;

        {
            CPU_PROFILE_BLOCK("Sort render passes");
            ZoneScopedN("RenderSystem::submit_renderer_params");

            frame_passes().opaque_render_pass.sort();
            frame_passes().transparent_render_pass.sort();
            renderer_holder_rc->ldr_render_pass->sort();

            for (uint32 i = 0; i < frame_passes().sun_shadow_render_pass.num_cascades; ++i)
            {
                frame_passes().sun_shadow_render_pass.cascades[i].sort();
            }

            for (uint32 i = 0; i < num_spot_light_shadow_passes; ++i)
            {
                frame_passes().spot_shadow_render_passes[i].sort();
            }

            for (uint32 i = 0; i < num_point_light_shadow_passes; ++i)
            {
                frame_passes().point_shadow_render_passes[i].sort();
            }
        }

        renderer->submit_render_pass(&frame_passes().opaque_render_pass);
        renderer->submit_render_pass(&frame_passes().transparent_render_pass);
        renderer->submit_render_pass(renderer_holder_rc->ldr_render_pass);
        renderer->submit_compute_pass(&frame_passes().compute_pass);

        renderer->submit_shadow_render_pass(&frame_passes().sun_shadow_render_pass);

        for (uint32 i = 0; i < num_spot_light_shadow_passes; ++i)
        {
            renderer->submit_shadow_render_pass(&frame_passes().spot_shadow_render_passes[i]);
        }

        for (uint32 i = 0; i < num_point_light_shadow_passes; ++i)
        {
            renderer->submit_shadow_render_pass(&frame_passes().point_shadow_render_passes[i]);
        }
    }
}