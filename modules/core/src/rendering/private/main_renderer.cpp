// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "rendering/main_renderer.h"
#include "rendering/raytracing.h"
#include "rendering/raytraced_reflections.h"

#include "core/log.h"
#include "core/job_system.h"

#include "dx/dx_command_list.h"
#include "dx/dx_render_target.h"
#include "dx/dx_pipeline.h"
#include "dx/dx_texture.h"
#include "dx/dx_barrier_batcher.h"
#include "dx/dx_profiling.h"

#include "particles/particles.h"

#include "light_source.hlsli"

#define SSR_RAYCAST_WIDTH (render_width / 2)
#define SSR_RAYCAST_HEIGHT (render_height / 2)

#define SSR_RESOLVE_WIDTH (render_width / 2)
#define SSR_RESOLVE_HEIGHT (render_height / 2)

namespace era_engine
{
    void MainRenderer::initialize(ColorDepth color_depth, uint32 window_width, uint32 window_height, RendererSpec spec, bool init_dlss)
    {
        this->window_width = window_width;
        this->window_height = window_height;
        const_cast<RendererSpec&>(this->spec) = spec;

        settings.enable_sharpen = spec.allow_taa;
        spec.allow_ssr |= dx_context.feature_support.raytracing(); // For now, since raytraced reflections use the same intermediate textures.

        recalculate_viewport(false);

        hdr_color_texture = create_texture(0, render_width, render_height, hdr_format, false, true, true, D3D12_RESOURCE_STATE_RENDER_TARGET);
        SET_NAME(hdr_color_texture->resource, "HDR Color");

        if (spec.allow_ssr)
        {
            D3D12_RESOURCE_DESC prev_frame_hdr_color_desc = CD3DX12_RESOURCE_DESC::Tex2D(hdr_format, render_width / 2, render_height / 2, 1,
                8, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
            prev_frame_hdr_color_texture = create_texture(prev_frame_hdr_color_desc, 0, 0, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, true);
            prev_frame_hdr_color_temp_texture = create_texture(prev_frame_hdr_color_desc, 0, 0, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, true);

            SET_NAME(prev_frame_hdr_color_texture->resource, "Prev frame HDR Color");
            SET_NAME(prev_frame_hdr_color_temp_texture->resource, "Prev frame HDR Color Temp");
        }

        // If object picking is allowed, but none of the others, we still have to allocate screen velocities due to dumb internal reasons with our render targets.
        if (spec.allow_ssr || spec.allow_taa || spec.allow_ao || spec.allow_object_picking)
        {
            screen_velocities_texture = create_texture(0, render_width, render_height, screen_velocities_format, false, true, false, D3D12_RESOURCE_STATE_GENERIC_READ);
            prev_frame_screen_velocities_texture = create_texture(0, render_width, render_height, screen_velocities_format, false, true, false, D3D12_RESOURCE_STATE_RENDER_TARGET);
            SET_NAME(screen_velocities_texture->resource, "Screen velocities 0");
            SET_NAME(prev_frame_screen_velocities_texture->resource, "Screen velocities 1");
        }

        world_normals_roughness_texture = create_texture(0, render_width, render_height, world_normals_roughness_format, false, true, false, D3D12_RESOURCE_STATE_GENERIC_READ);
        SET_NAME(world_normals_roughness_texture->resource, "World normals / roughness");

        depth_stencil_buffer = create_depth_texture(render_width, render_height, depth_stencil_format);
        opaque_depth_buffer = create_depth_texture(render_width, render_height, depth_stencil_format, 1, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        D3D12_RESOURCE_DESC linear_depth_desc = CD3DX12_RESOURCE_DESC::Tex2D(linear_depth_format, render_width, render_height, 1,
            6, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
        linear_depth_buffer = create_texture(linear_depth_desc, 0, 0, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, true);
        SET_NAME(depth_stencil_buffer->resource, "Depth buffer");
        SET_NAME(opaque_depth_buffer->resource, "Opaque depth buffer");
        SET_NAME(linear_depth_buffer->resource, "Linear depth buffer");

        if (spec.allow_ao)
        {
            ao_calculation_texture = create_texture(0, render_width / 2, render_height / 2, ao_format, false, false, true, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            ao_blur_temp_texture = create_texture(0, render_width, render_height, ao_format, false, false, true, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            ao_textures[ao_history_index] = create_texture(0, render_width, render_height, ao_format, false, false, true, D3D12_RESOURCE_STATE_GENERIC_READ);
            ao_textures[1 - ao_history_index] = create_texture(0, render_width, render_height, ao_format, false, false, true, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

            SET_NAME(ao_calculation_texture->resource, "AO Calculation");
            SET_NAME(ao_blur_temp_texture->resource, "AO Temp");
            SET_NAME(ao_textures[0]->resource, "AO 0");
            SET_NAME(ao_textures[1]->resource, "AO 1");
        }

        if (spec.allow_sss)
        {
            sss_calculation_texture = create_texture(0, render_width / 2, render_height / 2, sss_format, false, false, true, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            sss_blur_temp_texture = create_texture(0, render_width, render_height, sss_format, false, false, true, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            sss_textures[sss_history_index] = create_texture(0, render_width, render_height, sss_format, false, false, true, D3D12_RESOURCE_STATE_GENERIC_READ);
            sss_textures[1 - sss_history_index] = create_texture(0, render_width, render_height, sss_format, false, false, true, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

            SET_NAME(sss_calculation_texture->resource, "SSS Calculation");
            SET_NAME(sss_blur_temp_texture->resource, "SSS Temp");
            SET_NAME(sss_textures[0]->resource, "SSS 0");
            SET_NAME(sss_textures[1]->resource, "SSS 1");
        }

        if (spec.allow_ssr)
        {
            ssr_raycast_texture = create_texture(0, SSR_RAYCAST_WIDTH, SSR_RAYCAST_HEIGHT, hdr_format, false, false, true, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            ssr_resolve_texture = create_texture(0, SSR_RESOLVE_WIDTH, SSR_RESOLVE_HEIGHT, hdr_format, false, false, true, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            ssr_temporal_textures[ssr_history_index] = create_texture(0, SSR_RESOLVE_WIDTH, SSR_RESOLVE_HEIGHT, hdr_format, false, false, true, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
            ssr_temporal_textures[1 - ssr_history_index] = create_texture(0, SSR_RESOLVE_WIDTH, SSR_RESOLVE_HEIGHT, hdr_format, false, false, true, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

            SET_NAME(ssr_raycast_texture->resource, "SSR Raycast");
            SET_NAME(ssr_resolve_texture->resource, "SSR Resolve");
            SET_NAME(ssr_temporal_textures[0]->resource, "SSR Temporal 0");
            SET_NAME(ssr_temporal_textures[1]->resource, "SSR Temporal 1");
        }

        hdr_post_processing_texture = create_texture(0, render_width, render_height, hdr_format, false, true, true, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        SET_NAME(hdr_post_processing_texture->resource, "HDR Post processing");

        ldr_post_processing_texture = create_texture(0, render_width, render_height, ldr_format, false, true, true, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        SET_NAME(ldr_post_processing_texture->resource, "LDR Post processing");

        if (spec.allow_taa)
        {
            taa_textures[taa_history_index] = create_texture(0, render_width, render_height, hdr_format, false, true, true, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
            taa_textures[1 - taa_history_index] = create_texture(0, render_width, render_height, hdr_format, false, true, true, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

            SET_NAME(taa_textures[0]->resource, "TAA 0");
            SET_NAME(taa_textures[1]->resource, "TAA 1");
        }

        if (spec.allow_bloom)
        {
            D3D12_RESOURCE_DESC bloom_desc = CD3DX12_RESOURCE_DESC::Tex2D(hdr_format, render_width / 4, render_height / 4, 1,
                5, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
            bloom_texture = create_texture(bloom_desc, 0, 0, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, true);
            bloom_temp_texture = create_texture(bloom_desc, 0, 0, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, true);

            SET_NAME(bloom_texture->resource, "Bloom");
            SET_NAME(bloom_temp_texture->resource, "Bloom Temp");
        }

        frame_result = create_texture(0, window_width, window_height, color_depth_to_format(color_depth), false, true, true);
        SET_NAME(frame_result->resource, "Frame result");

        if (spec.allow_object_picking)
        {
            hovered_object_id_readback_buffer = create_readback_buffer(get_format_size(object_ids_format), NUM_BUFFERED_FRAMES);

            object_ids_texture = create_texture(0, render_width, render_height, object_ids_format, false, true, false, D3D12_RESOURCE_STATE_RENDER_TARGET);
            SET_NAME(object_ids_texture->resource, "Object IDs");
        }

        if (dx_context.feature_support.raytracing())
        {
            path_tracer.initialize();
        }

        if (init_dlss && dx_context.feature_support.dlss())
        {
            dlss_adapter.initialize(this);
        }
    }

    MainRendererFrameInputs& MainRenderer::collected()
    {
        return frame_inputs[dx_context.get_buffered_frame_id()];
    }

    void MainRenderer::begin_frame(uint32 window_width, uint32 window_height)
    {
        MainRendererFrameInputs& inputs = collected();
        inputs = MainRendererFrameInputs{};
        inputs.requested_width = window_width;
        inputs.requested_height = window_height;

        if (object_ids_texture && window_hovered)
        {
            uint32* id = (uint32*)map_buffer(hovered_object_id_readback_buffer, true, MapRange{dx_context.get_buffered_frame_id(), 1});
            hovered_object_id = *id;
            unmap_buffer(hovered_object_id_readback_buffer, false);
        }
        else
        {
            hovered_object_id = -1;
        }
    }

    void MainRenderer::end_collection()
    {
        MainRendererFrameInputs& inputs = collected();
        inputs.frame_settings = settings;
        inputs.frame_mode = mode;
        inputs.frame_aspect_ratio_mode = aspect_ratio_mode;
    }

    void MainRenderer::recalculate_viewport(bool resize_textures)
    {
        if (frame_aspect_ratio_mode == ASPECT_RATIO_FREE)
        {
            render_width = window_width;
            render_height = window_height;
            window_x_offset = 0;
            window_y_offset = 0;
        }
        else
        {
            const float target_aspect = frame_aspect_ratio_mode == ASPECT_RATIO_FIX_16_9 ? (16.f / 9.f) : (16.f / 10.f);

            float aspect = (float)window_width / (float)window_height;
            if (aspect > target_aspect)
            {
                render_width = (uint32)(window_height * target_aspect);
                render_height = window_height;
                window_x_offset = (int32)(window_width - render_width) / 2;
                window_y_offset = 0;
            }
            else
            {
                render_width = window_width;
                render_height = (uint32)(window_width / target_aspect);
                window_x_offset = 0;
                window_y_offset = (int32)(window_height - render_height) / 2;
            }
        }

        if (resize_textures)
        {
            resize_texture(hdr_color_texture, render_width, render_height);
            resize_texture(prev_frame_hdr_color_texture, render_width / 2, render_height / 2);
            resize_texture(prev_frame_hdr_color_temp_texture, render_width / 2, render_height / 2);
            resize_texture(world_normals_roughness_texture, render_width, render_height);
            resize_texture(screen_velocities_texture, render_width, render_height, D3D12_RESOURCE_STATE_GENERIC_READ);
            resize_texture(prev_frame_screen_velocities_texture, render_width, render_height, D3D12_RESOURCE_STATE_RENDER_TARGET);
            resize_texture(depth_stencil_buffer, render_width, render_height);
            resize_texture(opaque_depth_buffer, render_width, render_height);
            resize_texture(linear_depth_buffer, render_width, render_height);

            resize_texture(ao_calculation_texture, render_width / 2, render_height / 2);
            resize_texture(ao_blur_temp_texture, render_width, render_height);
            resize_texture(ao_textures[ao_history_index], render_width, render_height, D3D12_RESOURCE_STATE_GENERIC_READ);
            resize_texture(ao_textures[1 - ao_history_index], render_width, render_height, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

            resize_texture(sss_calculation_texture, render_width / 2, render_height / 2);
            resize_texture(sss_blur_temp_texture, render_width, render_height);
            resize_texture(sss_textures[sss_history_index], render_width, render_height, D3D12_RESOURCE_STATE_GENERIC_READ);
            resize_texture(sss_textures[1 - sss_history_index], render_width, render_height, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

            resize_texture(object_ids_texture, render_width, render_height);

            resize_texture(ssr_raycast_texture, SSR_RAYCAST_WIDTH, SSR_RAYCAST_HEIGHT);
            resize_texture(ssr_resolve_texture, SSR_RESOLVE_WIDTH, SSR_RESOLVE_HEIGHT);
            resize_texture(ssr_temporal_textures[ssr_history_index], SSR_RESOLVE_WIDTH, SSR_RESOLVE_HEIGHT, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
            resize_texture(ssr_temporal_textures[1 - ssr_history_index], SSR_RESOLVE_WIDTH, SSR_RESOLVE_HEIGHT, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

            resize_texture(hdr_post_processing_texture, render_width, render_height);

            resize_texture(taa_textures[taa_history_index], render_width, render_height, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
            resize_texture(taa_textures[1 - taa_history_index], render_width, render_height, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

            resize_texture(bloom_texture, render_width / 4, render_height / 4);
            resize_texture(bloom_temp_texture, render_width / 4, render_height / 4);

            resize_texture(ldr_post_processing_texture, render_width, render_height);
        }

        culling.allocate_if_necessary(render_width, render_height);
    }

    uint32 MainRenderer::get_num_available_modes()
    {
        return dx_context.feature_support.raytracing() ? RENDERER_MODE_COUNT : RENDERER_MODE_PATHTRACED;
    }

    void MainRenderer::set_camera(const RenderCamera& camera)
    {
        // Builds on the previous frame's buffers, so they become the prev_frame fields.
        build_camera_constant_buffer(camera, settings.taa_settings.camera_jitter_strength * settings.enable_taa, camera_history_jittered);
        build_camera_constant_buffer(camera, 0.f, camera_history_unjittered);

        collected().jittered_camera = camera_history_jittered;
        collected().unjittered_camera = camera_history_unjittered;
    }

    void MainRenderer::dlss_pass(float dt)
    {
        dlss_adapter.update_dlss(dx_context.get_free_render_command_list()->command_list.Get(), dt);
    }

    void MainRenderer::set_environment(const PBREnvironment& environment)
    {
        collected().environment = &environment;
    }

    void MainRenderer::set_sun(const DirectionalLight& light)
    {
        DirectionalLightCB& sun = collected().sun;
        sun.cascade_distances = light.cascade_distances;
        sun.bias = light.depth_bias;
        sun.direction = light.direction;
        sun.blend_distances = light.blend_distances;
        sun.radiance = light.color * light.intensity;
        sun.num_shadow_cascades = light.num_shadow_cascades;

        memcpy(sun.view_projs, light.view_projs, sizeof(mat4) * light.num_shadow_cascades);
        memcpy(sun.viewports, light.shadow_map_viewports, sizeof(vec4) * light.num_shadow_cascades);
    }

    void MainRenderer::set_point_lights(const ref<DXBuffer>& lights, uint32 num_lights, const ref<DXBuffer>& shadow_info_buffer)
    {
        MainRendererFrameInputs& inputs = collected();
        inputs.point_lights = lights;
        inputs.num_point_lights = num_lights;
        inputs.point_light_shadow_info_buffer = shadow_info_buffer;
    }

    void MainRenderer::set_spot_lights(const ref<DXBuffer>& lights, uint32 num_lights, const ref<DXBuffer>& shadow_info_buffer)
    {
        MainRendererFrameInputs& inputs = collected();
        inputs.spot_lights = lights;
        inputs.num_spot_lights = num_lights;
        inputs.spot_light_shadow_info_buffer = shadow_info_buffer;
    }

    void MainRenderer::set_decals(const ref<DXBuffer>& decals, uint32 num_decals, const ref<DXTexture>& texture_atlas)
    {
        ASSERT(num_decals < MAX_NUM_TOTAL_DECALS);
        MainRendererFrameInputs& inputs = collected();
        inputs.decals = decals;
        inputs.num_decals = num_decals;
        inputs.decal_texture_atlas = texture_atlas;
    }

    template <typename func_t>
    uint64 MainRenderer::execute_compute_tasks(ComputePassEvent event_time, const func_t& additional_tasks_callback)
    {
        uint64 result = 0;

        bool has_compute_pass_events = compute_pass && compute_pass->passes[event_time].size() > 0;
        bool has_additional_tasks = !std::is_null_pointer_v<func_t>;

        if (has_compute_pass_events || has_additional_tasks)
        {
            DXCommandList* cl = dx_context.get_free_compute_command_list(true);

            if (has_compute_pass_events)
            {
                for (const auto& dc : compute_pass->passes[event_time])
                {
                    dc.compute(cl, dc.data);
                }
            }

            if constexpr (!std::is_null_pointer_v<func_t>)
            {
                additional_tasks_callback(cl);
            }

            result = dx_context.execute_command_list(cl);
        }

        return result;
    }

    NODISCARD DXCommandList* MainRenderer::render_thread0(const CommonRenderData& common_render_data)
    {
        ZoneScopedN("MainRenderer::render_thread0");

        DXCommandList* cl = dx_context.get_free_render_command_list();
        PROFILE_ALL(cl, "Render thread 0");

        {
            PROFILE_ALL(cl, "Shadow maps");

            cl->set_primitive_topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

            cl->transition_barrier(RenderResources::shadow_map, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_DEPTH_WRITE);

            shadow_passes(cl, sun_shadow_render_passes, num_sun_light_shadow_render_passes,
                spot_light_shadow_render_passes, num_spot_light_shadow_render_passes,
                point_light_shadow_render_passes, num_point_light_shadow_render_passes,
                common_render_data);

            cl->transition_barrier(RenderResources::shadow_map, D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        }

        return cl;
    }

    NODISCARD DXCommandList* MainRenderer::render_thread1(const CommonRenderData& common_render_data, bool aspect_ratio_mode_changed)
    {
        ZoneScopedN("MainRenderer::render_thread1");

        DXCommandList* cl = dx_context.get_free_render_command_list();
        PROFILE_ALL(cl, "Render thread 1");

        if (aspect_ratio_mode_changed)
        {
            cl->transition_barrier(frame_result, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_RENDER_TARGET);
            cl->clear_rtv(frame_result, 0.f, 0.f, 0.f);
            cl->transition_barrier(frame_result, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COMMON);
        }

        cl->clear_depth_and_stencil(depth_stencil_buffer);

        // ----------------------------------------
        // DEPTH-ONLY PASS
        // ----------------------------------------
        auto depth_only_render_target = DXRenderTarget(render_width, render_height)
                                            .color_attachment(screen_velocities_texture, RenderResources::null_screen_velocities_rtv)
                                            .color_attachment(object_ids_texture, RenderResources::null_object_ids_rtv)
                                            .depth_attachment(depth_stencil_buffer);

        depth_pre_pass(cl, depth_only_render_target, opaque_render_pass,
            jittered_camera.view_proj, jittered_camera.prev_frame_view_proj, common_render_data);

        BarrierBatcher(cl)
            .transition(depth_stencil_buffer, D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

        // ----------------------------------------
        // LIGHT & DECAL CULLING
        // ----------------------------------------
        light_and_decal_culling(cl, depth_stencil_buffer, point_lights, spot_lights, decals, culling, num_point_lights, num_spot_lights, num_decals, common_render_data.camera_cbv);

        // ---------------------------------------
        // LINEAR DEPTH PYRAMID
        // ---------------------------------------
        linear_depth_pyramid(cl, depth_stencil_buffer, linear_depth_buffer, jittered_camera.projection_params);

        BarrierBatcher(cl)
            //.uav(linearDepthBuffer)
            .transition(linear_depth_buffer, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
            .transition(depth_stencil_buffer, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_DEPTH_WRITE);

        return cl;
    }

    NODISCARD DXCommandList* MainRenderer::render_thread2(const CommonRenderData& common_render_data, const UserInput* input)
    {
        ZoneScopedN("MainRenderer::render_thread2");

        DXCommandList* cl = dx_context.get_free_render_command_list();
        PROFILE_ALL(cl, "Render thread 2");

        // ----------------------------------------
        // SKY PASS
        // ----------------------------------------

        auto sky_render_target = DXRenderTarget(render_width, render_height)
                                     .color_attachment(hdr_color_texture)
                                     .color_attachment(screen_velocities_texture, RenderResources::null_screen_velocities_rtv)
                                     .color_attachment(object_ids_texture, RenderResources::null_object_ids_rtv)
                                     .depth_attachment(depth_stencil_buffer);

        if (environment && !environment->is_procedural())
        {
            ASSERT(environment->sky);

            textured_sky(cl, sky_render_target, jittered_camera.proj, jittered_camera.view, jittered_camera.prev_frame_view, environment->sky, environment->sky_intensity,
                jittered_camera.jitter, jittered_camera.prev_frame_jitter);
        }
        else
        {
            procedural_sky(cl, sky_render_target, jittered_camera.proj, jittered_camera.view, jittered_camera.prev_frame_view, sun.direction, environment ? environment->sky_intensity : 1.f,
                jittered_camera.jitter, jittered_camera.prev_frame_jitter);
        }

        BarrierBatcher(cl)
            .transition(screen_velocities_texture, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_GENERIC_READ);

        // ----------------------------------------
        // AMBIENT OCCLUSION
        // ----------------------------------------

        if (frame_settings.enable_ao)
        {
            uint32 ao_result_index = 1 - ao_history_index;
            ref<DXTexture> ao_history = ao_was_on_last_frame ? ao_textures[ao_history_index] : RenderResources::white_texture;
            ref<DXTexture> ao_result = ao_textures[ao_result_index];
            ambient_occlusion(cl, linear_depth_buffer, screen_velocities_texture, ao_calculation_texture, ao_blur_temp_texture, ao_history, ao_result, frame_settings.ao_settings, common_render_data.camera_cbv);

            BarrierBatcher(cl)
                // UAV barrier is done by hbao-function.
                .transition(ao_textures[ao_result_index], D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_GENERIC_READ)   // Read by opaque pass and next frame as history.
                .transition(ao_textures[ao_history_index], D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_STATE_UNORDERED_ACCESS); // For next frame.

            ao_history_index = ao_result_index;
        }

        // ----------------------------------------
        // SCREEN SPACE REFLECTIONS
        // ----------------------------------------

        if (environment && environment->gi_mode == ENVIRONMENT_GI_RAYTRACED)
        {
            BarrierBatcher(cl)
                .transition(depth_stencil_buffer, D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

            PROFILE_ALL(cl, "Specular reflections");
            raytrace_rt_reflections(cl, *tlas, depth_stencil_buffer, world_normals_roughness_texture,
                screen_velocities_texture, ssr_raycast_texture, ssr_resolve_texture, ssr_temporal_textures[ssr_history_index],
                ssr_temporal_textures[1 - ssr_history_index], common_render_data);

            ssr_history_index = 1 - ssr_history_index;

            BarrierBatcher(cl)
                .transition(depth_stencil_buffer, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_DEPTH_WRITE);
        }
        else if (frame_settings.enable_ssr)
        {
            BarrierBatcher(cl)
                .transition(depth_stencil_buffer, D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

            screen_space_reflections(cl, prev_frame_hdr_color_texture, depth_stencil_buffer, linear_depth_buffer, world_normals_roughness_texture,
                screen_velocities_texture, prev_frame_screen_velocities_texture, ssr_raycast_texture, ssr_resolve_texture, ssr_temporal_textures[ssr_history_index],
                ssr_temporal_textures[1 - ssr_history_index], frame_settings.ssr_settings, common_render_data.camera_cbv);

            ssr_history_index = 1 - ssr_history_index;

            BarrierBatcher(cl)
                .transition(depth_stencil_buffer, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_DEPTH_WRITE);
        }

        // ----------------------------------------
        // SCREEN SPACE SHADOWS
        // ----------------------------------------

        if (frame_settings.enable_sss)
        {
            uint32 sss_result_index = 1 - sss_history_index;
            ref<DXTexture> sss_history = sss_was_on_last_frame ? sss_textures[sss_history_index] : RenderResources::white_texture;
            ref<DXTexture> sss_result = sss_textures[sss_result_index];
            screen_space_shadows(cl, linear_depth_buffer, screen_velocities_texture, sss_calculation_texture, sss_blur_temp_texture, sss_history, sss_result, sun.direction, frame_settings.sss_settings, jittered_camera.view, common_render_data.camera_cbv);

            BarrierBatcher(cl)
                // UAV barrier is done by sss-function.
                .transition(sss_textures[sss_result_index], D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_GENERIC_READ)   // Read by opaque pass and next frame as history.
                .transition(sss_textures[sss_history_index], D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_STATE_UNORDERED_ACCESS); // For next frame.

            sss_history_index = sss_result_index;
        }

        // ----------------------------------------
        // OPAQUE LIGHT PASS
        // ----------------------------------------

        BarrierBatcher(cl)
            .transition(world_normals_roughness_texture, D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_STATE_RENDER_TARGET)
            .transition(screen_velocities_texture, D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_STATE_RENDER_TARGET);

        auto hdr_opaque_render_target = DXRenderTarget(render_width, render_height)
                                            .color_attachment(hdr_color_texture)
                                            .color_attachment(world_normals_roughness_texture)
                                            .color_attachment(screen_velocities_texture, RenderResources::null_screen_velocities_rtv)
                                            .color_attachment(object_ids_texture, RenderResources::null_object_ids_rtv)
                                            .depth_attachment(depth_stencil_buffer);

        opaque_light_pass(cl, hdr_opaque_render_target, opaque_render_pass, common_render_data, jittered_camera.view_proj);

        BarrierBatcher(cl)
            .transition(hdr_color_texture, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
            .transition(depth_stencil_buffer, D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_GENERIC_READ)
            .transition(world_normals_roughness_texture, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_GENERIC_READ)
            .transition(screen_velocities_texture, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_GENERIC_READ)
            .transition(frame_result, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

        // Copy hovered object id to readback buffer.
        if (object_ids_texture && input)
        {
            window_hovered = input->over_window && (input->mouse.x - window_x_offset >= 0) && (input->mouse.x - window_x_offset < (int32)render_width) && (input->mouse.y - window_y_offset >= 0) && (input->mouse.y - window_y_offset < (int32)render_height);

            if (window_hovered)
            {
                PROFILE_ALL(cl, "Copy hovered object ID");

                cl->transition_barrier(object_ids_texture, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_SOURCE);
                cl->copy_texture_region_to_buffer(object_ids_texture, hovered_object_id_readback_buffer, dx_context.get_buffered_frame_id(), input->mouse.x - window_x_offset, input->mouse.y - window_y_offset, 1, 1);
                cl->transition_barrier(object_ids_texture, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
            }
        }

        return cl;
    }

    NODISCARD DXCommandList* MainRenderer::render_thread3(CommonRenderData common_render_data, DXDynamicConstantBuffer unjittered_camera_cbv)
    {
        ZoneScopedN("MainRenderer::render_thread3");

        // After this there is no more camera jittering
        common_render_data.camera_cbv = unjittered_camera_cbv;

        DXCommandList* cl = dx_context.get_free_render_command_list();
        PROFILE_ALL(cl, "Render thread 3");

        BarrierBatcher(cl)
            .transition(opaque_depth_buffer, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_COPY_DEST);

        {
            PROFILE_ALL(cl, "Copy depth buffer");
            cl->copy_resource(depth_stencil_buffer->resource, opaque_depth_buffer->resource);
        }

        BarrierBatcher(cl)
            .transition(opaque_depth_buffer, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);

        ref<DXTexture> hdr_result = hdr_color_texture; // Read state

        // ---------------------------------------
        // TRANSPARENT LIGHT PASS
        // ---------------------------------------

        if (transparent_render_pass && transparent_render_pass->pass.size() > 0)
        {
            // There are some really ugly barriers in this scope

            BarrierBatcher(cl)
                .transition(hdr_post_processing_texture, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_COPY_DEST)
                .transition(hdr_result, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_COPY_SOURCE);

            {
                DX_PROFILE_BLOCK(cl, "Copy opaque color");
                cl->copy_resource(hdr_result->resource, hdr_post_processing_texture->resource);
            }

            BarrierBatcher(cl)
                .transition(hdr_post_processing_texture, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE)
                .transition(hdr_result, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET)
                .transition(depth_stencil_buffer, D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_STATE_DEPTH_WRITE);

            auto hdr_transparent_render_target = DXRenderTarget(render_width, render_height)
                                                     .color_attachment(hdr_result)
                                                     .depth_attachment(depth_stencil_buffer);

            transparent_light_pass(cl, hdr_transparent_render_target, transparent_render_pass, common_render_data, unjittered_camera.view_proj);

            if (screen_velocities_texture && transparent_render_pass->motion_vectors.size() > 0)
            {
                cl->transition_barrier(screen_velocities_texture, D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_STATE_RENDER_TARGET);

                auto motion_vectors_render_target = DXRenderTarget(render_width, render_height)
                                                        .color_attachment(screen_velocities_texture)
                                                        .depth_attachment(depth_stencil_buffer);

                transparent_motion_vectors_pass(cl, motion_vectors_render_target, transparent_render_pass,
                    unjittered_camera.view_proj, unjittered_camera.prev_frame_view_proj, common_render_data);

                cl->transition_barrier(screen_velocities_texture, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_GENERIC_READ);
            }

            BarrierBatcher(cl)
                .transition(hdr_post_processing_texture, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
                .transition(hdr_result, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
                .transition(depth_stencil_buffer, D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_GENERIC_READ);
        }

        if ((environment && environment->gi_mode == ENVIRONMENT_GI_RAYTRACED) || frame_settings.enable_ssr)
        {
            BarrierBatcher(cl)
                .transition(ssr_resolve_texture, D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_STATE_UNORDERED_ACCESS); // For next frame.
        }

        // ----------------------------------------
        // POST PROCESSING
        // ----------------------------------------

        // TAA
        if (frame_settings.enable_taa)
        {
            uint32 taa_output_index = 1 - taa_history_index;
            temporal_anti_aliasing(cl, hdr_result, screen_velocities_texture, prev_frame_screen_velocities_texture, opaque_depth_buffer, taa_textures[taa_history_index], taa_textures[taa_output_index], jittered_camera.projection_params);
            hdr_result = taa_textures[taa_output_index];
            taa_history_index = taa_output_index;
        }

        // At this point hdrResult is either the TAA result or the hdrResult. Either one is in read state.
        // Downsample scene. This is also the copy used in SSR next frame.
        if (prev_frame_hdr_color_texture)
        {
            downsample(cl, hdr_result, prev_frame_hdr_color_texture, prev_frame_hdr_color_temp_texture);
        }

        // Bloom
        if (frame_settings.enable_bloom)
        {
            BarrierBatcher(cl)
                .transition(hdr_post_processing_texture, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

            bloom(cl, hdr_result, hdr_post_processing_texture, bloom_texture, bloom_temp_texture, frame_settings.bloom_settings); // Bloom returns result in read state.
            hdr_result = hdr_post_processing_texture;
        }

        // At this point hdrResult is either the TAA result, the hdrColorTexture, or the hdrPostProcessingTexture. Either one is in read state.
        tonemap(cl, hdr_result, ldr_post_processing_texture, frame_settings.tonemap_settings);

        // ----------------------------------------
        // LDR RENDERING
        // ----------------------------------------

        bool rendering_ldr = ldr_render_pass && ldr_render_pass->ldr_pass.size();
        bool rendering_overlays = ldr_render_pass && ldr_render_pass->overlays.size();
        bool rendering_outlines = ldr_render_pass && ldr_render_pass->outlines.size();
        if (rendering_ldr || rendering_overlays || rendering_outlines)
        {
            BarrierBatcher(cl)
                //.uav(ldrPostProcessingTexture)
                .transition(ldr_post_processing_texture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_RENDER_TARGET)
                .transition(depth_stencil_buffer, D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_STATE_DEPTH_WRITE);

            auto ldr_render_target = DXRenderTarget(render_width, render_height)
                                         .color_attachment(ldr_post_processing_texture)
                                         .depth_attachment(depth_stencil_buffer);

            ldr_pass(cl, ldr_render_target, depth_stencil_buffer, ldr_render_pass, common_render_data, unjittered_camera.view_proj);

            BarrierBatcher(cl)
                .transition(ldr_post_processing_texture, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        }
        else
        {
            BarrierBatcher(cl)
                .transition(ldr_post_processing_texture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
                .transition(depth_stencil_buffer, D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_STATE_DEPTH_WRITE);
        }

        present(cl, ldr_post_processing_texture, frame_result, frame_settings.enable_sharpen ? frame_settings.sharpen_settings : SharpenSettings{0.f});

        BarrierBatcher(cl)
            //.uav(frameResult)
            .transition(hdr_color_texture, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET)
            .transition(prev_frame_screen_velocities_texture, D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_STATE_RENDER_TARGET) // Written next frame.
            .transition(ldr_post_processing_texture, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
            .transition(frame_result, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COMMON)
            .transition(linear_depth_buffer, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

        return cl;
    }

    void MainRenderer::end_frame(const UserInput* input)
    {
        ZoneScopedN("MainRenderer::end_frame");

        static_cast<MainRendererFrameInputs&>(*this) = collected();

        if (requested_width != 0 && requested_height != 0 && (window_width != requested_width || window_height != requested_height))
        {
            window_width = requested_width;
            window_height = requested_height;

            resize_texture(frame_result, window_width, window_height);

            recalculate_viewport(true);
        }

        if (frame_mode == RENDERER_MODE_PATHTRACED && !dx_context.feature_support.raytracing())
        {
            LOG_ERROR("Path tracing requires DXR support, falling back to rasterization");
            frame_mode = RENDERER_MODE_RASTERIZED;
        }

        bool aspect_ratio_mode_changed = frame_aspect_ratio_mode != old_aspect_ratio_mode;
        old_aspect_ratio_mode = frame_aspect_ratio_mode;

        if (aspect_ratio_mode_changed)
        {
            recalculate_viewport(true);
        }

        if (frame_mode == RENDERER_MODE_RASTERIZED && screen_velocities_texture)
        {
            std::swap(screen_velocities_texture, prev_frame_screen_velocities_texture);
        }

        auto jittered_camera_cbv = dx_context.upload_dynamic_constant_buffer(jittered_camera);
        auto unjittered_camera_cbv = dx_context.upload_dynamic_constant_buffer(unjittered_camera);

        frame_settings.enable_ao &= spec.allow_ao;
        frame_settings.enable_sss &= spec.allow_sss;
        frame_settings.enable_ssr &= spec.allow_ssr;
        frame_settings.enable_bloom &= spec.allow_bloom;
        frame_settings.enable_taa &= spec.allow_taa;

        LightingCB lighting_cb =
            {
                sun,
                environment ? environment->light_probe_grid.get_cb() : LightProbeGridCB{},
                vec2(1.f / SHADOW_MAP_WIDTH, 1.f / SHADOW_MAP_HEIGHT),
                environment ? environment->global_illumination_intensity : 1.f,
                (environment && environment->gi_mode == ENVIRONMENT_GI_RAYTRACED),
            };

        DXDynamicConstantBuffer lighting_cbv = dx_context.upload_dynamic_constant_buffer(lighting_cb);

        CommonRenderData common_render_data;
        common_render_data.lighting_cbv = lighting_cbv;
        common_render_data.procedural_sky = !(environment && environment->sky);
        common_render_data.sky = (environment && environment->sky) ? environment->sky : RenderResources::black_cube_texture;
        common_render_data.irradiance = (environment && environment->irradiance) ? environment->irradiance : RenderResources::black_cube_texture;
        common_render_data.prefiltered_radiance = (environment && environment->prefiltered_radiance) ? environment->prefiltered_radiance : RenderResources::black_cube_texture;

        common_render_data.global_illumination_intensity = environment ? environment->global_illumination_intensity : 1.f;
        common_render_data.sky_intensity = environment ? environment->sky_intensity : 1.f;
        common_render_data.ao_texture = frame_settings.enable_ao ? ao_textures[1 - ao_history_index] : RenderResources::white_texture;
        common_render_data.sss_texture = frame_settings.enable_sss ? sss_textures[1 - sss_history_index] : RenderResources::white_texture;
        common_render_data.ssr_texture = frame_settings.enable_ssr ? ssr_resolve_texture : 0;
        common_render_data.tiled_culling_grid = culling.tiled_culling_grid;
        common_render_data.tiled_objects_index_list = culling.tiled_objects_index_list;
        common_render_data.point_light_buffer = point_lights;
        common_render_data.spot_light_buffer = spot_lights;
        common_render_data.decal_buffer = decals;
        common_render_data.shadow_map = RenderResources::shadow_map;
        common_render_data.decal_texture_atlas = decal_texture_atlas;
        common_render_data.point_light_shadow_info_buffer = point_light_shadow_info_buffer;
        common_render_data.spot_light_shadow_info_buffer = spot_light_shadow_info_buffer;
        common_render_data.volumetrics_texture = 0;
        common_render_data.camera_cbv = jittered_camera_cbv;

        common_render_data.light_probe_irradiance = (environment && environment->gi_mode == ENVIRONMENT_GI_RAYTRACED) ? environment->light_probe_grid.irradiance : 0;
        common_render_data.light_probe_depth = (environment && environment->gi_mode == ENVIRONMENT_GI_RAYTRACED) ? environment->light_probe_grid.depth : 0;
        common_render_data.opaque_color = hdr_post_processing_texture;
        common_render_data.opaque_depth = opaque_depth_buffer;
        common_render_data.world_normals_and_roughness = world_normals_roughness_texture;

        common_render_data.camera_jitter = jittered_camera.jitter;
        common_render_data.prev_frame_camera_jitter = jittered_camera.prev_frame_jitter;
        CommonParticleSimulationData common_particle_data;

        common_particle_data.camera_position = unjittered_camera.position.xyz;
        common_particle_data.prev_frame_camera_view = unjittered_camera.prev_frame_view;
        common_particle_data.prev_frame_camera_view_proj = unjittered_camera.prev_frame_view_proj;
        common_particle_data.camera_projection_params = unjittered_camera.projection_params;
        common_particle_data.prev_frame_depth_buffer = opaque_depth_buffer;
        common_particle_data.prev_frame_normals = world_normals_roughness_texture;

        if (frame_mode == RENDERER_MODE_RASTERIZED)
        {
            struct RenderJobData
            {
                MainRenderer* renderer;
                const CommonRenderData& common_render_data;

                bool aspect_ratio_mode_changed;
                const UserInput* input;
                DXDynamicConstantBuffer unjittered_camera_cbv;

                DXCommandList* cl0 = nullptr;
                DXCommandList* cl1 = nullptr;
                DXCommandList* cl2 = nullptr;
                DXCommandList* cl3 = nullptr;
            } job_data{this, common_render_data, aspect_ratio_mode_changed, input, unjittered_camera_cbv};

            const JobDecl render_jobs[] =
                {
                    JobDecl([](void* data)
                        {
                            RenderJobData& job = *static_cast<RenderJobData*>(data);
                            job.cl0 = job.renderer->render_thread0(job.common_render_data);
                        },
                        &job_data, JobPriority::HIGH, false, "MainRenderer::render_thread0"),
                    JobDecl([](void* data)
                        {
                            RenderJobData& job = *static_cast<RenderJobData*>(data);
                            job.cl1 = job.renderer->render_thread1(job.common_render_data, job.aspect_ratio_mode_changed);
                        },
                        &job_data, JobPriority::HIGH, false, "MainRenderer::render_thread1"),
                    JobDecl([](void* data)
                        {
                            RenderJobData& job = *static_cast<RenderJobData*>(data);
                            job.cl2 = job.renderer->render_thread2(job.common_render_data, job.input);
                        },
                        &job_data, JobPriority::HIGH, false, "MainRenderer::render_thread2"),
                    JobDecl([](void* data)
                        {
                            RenderJobData& job = *static_cast<RenderJobData*>(data);
                            job.cl3 = job.renderer->render_thread3(job.common_render_data, job.unjittered_camera_cbv);
                        },
                        &job_data, JobPriority::HIGH, false, "MainRenderer::render_thread3"),
                };

            JobCounter render_jobs_counter;
            JobSystem::run_jobs(render_jobs, (uint32)std::size(render_jobs), &render_jobs_counter);
            JobSystem::wait_for_counter(&render_jobs_counter);

            DXCommandList* cl0 = job_data.cl0;
            DXCommandList* cl1 = job_data.cl1;
            DXCommandList* cl2 = job_data.cl2;
            DXCommandList* cl3 = job_data.cl3;

            ASSERT(cl0);
            ASSERT(cl1);
            ASSERT(cl2);
            ASSERT(cl3);

            dx_context.render_queue.wait_for_other_queue(dx_context.compute_queue, execute_compute_tasks(COMPUTE_PASS_FRAME_START));
            dx_context.execute_command_list(cl0);

            dx_context.render_queue.wait_for_other_queue(dx_context.compute_queue, execute_compute_tasks(COMPUTE_PASS_BEFORE_DEPTH_PREPASS));
            dx_context.execute_command_list(cl1);

            dx_context.render_queue.wait_for_other_queue(dx_context.compute_queue, execute_compute_tasks(COMPUTE_PASS_BEFORE_OPAQUE,
                                                                                       [this, &common_render_data, &common_particle_data](DXCommandList* cl)
                                                                                       {
                                                                                           if (dx_context.feature_support.raytracing() && tlas)
                                                                                           {
                                                                                               tlas->build(cl);

                                                                                               if (environment && environment->gi_mode == ENVIRONMENT_GI_RAYTRACED)
                                                                                               {
                                                                                                   PROFILE_ALL(cl, "Update light probes");
                                                                                                   environment->light_probe_grid.update_probes(cl, *tlas, common_render_data);
                                                                                               }
                                                                                           }

                                                                                           if (compute_pass && compute_pass->particle_system_updates.size() > 0)
                                                                                           {
                                                                                               for (auto& cc : compute_pass->particle_system_updates)
                                                                                               {
                                                                                                   cc->update(cl, common_particle_data, compute_pass->dt);
                                                                                               }
                                                                                           }
                                                                                       }));

            dx_context.execute_command_list(cl2);

            dx_context.render_queue.wait_for_other_queue(dx_context.compute_queue, execute_compute_tasks(COMPUTE_PASS_BEFORE_TRANSPARENT_AND_POST_PROCESSING));

            if (dx_context.feature_support.dlss() && frame_settings.enable_dlss)
            {
                PROFILE_ALL(cl3, "DLSS Evaluation");
                dlss_adapter.update_dlss(cl3->command_list.Get(), compute_pass->dt);
            }

            dx_context.execute_command_list(cl3);
        }
        else if (frame_mode == RENDERER_MODE_VISUALIZE_SUN_SHADOW_CASCADES)
        {
            uint64 max_fence = 0;
            for (uint32 i = 0; i < COMPUTE_PASS_EVENT_COUNT; ++i)
            {
                uint64 fence = execute_compute_tasks((ComputePassEvent)i);
                max_fence = max(max_fence, fence);
            }

            dx_context.render_queue.wait_for_other_queue(dx_context.compute_queue, max_fence);

            DXCommandList* cl = dx_context.get_free_render_command_list();

            if (aspect_ratio_mode_changed)
            {
                cl->transition_barrier(frame_result, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_RENDER_TARGET);
                cl->clear_rtv(frame_result, 0.f, 0.f, 0.f);
                cl->transition_barrier(frame_result, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COMMON);
            }

            cl->clear_depth_and_stencil(depth_stencil_buffer);

            auto& depth_only_render_target = DXRenderTarget(render_width, render_height)
                                                 // No screen space velocities or object IDs needed.
                                                 .depth_attachment(depth_stencil_buffer);

            depth_pre_pass(cl, depth_only_render_target, opaque_render_pass,
                unjittered_camera.view_proj, unjittered_camera.prev_frame_view_proj, common_render_data);

            BarrierBatcher(cl)
                .transition(depth_stencil_buffer, D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
                .transition(frame_result, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

            visualize_sun_shadow_cascades(cl, depth_stencil_buffer, frame_result, lighting_cbv, unjittered_camera.inv_view_proj, unjittered_camera.position.xyz, unjittered_camera.forward.xyz);

            BarrierBatcher(cl)
                //.uav(frameResult)
                .transition(depth_stencil_buffer, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_DEPTH_WRITE)
                .transition(frame_result, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COMMON);

            dx_context.execute_command_list(cl);
        }
        else if (frame_mode == RENDERER_MODE_PATHTRACED && dx_context.feature_support.raytracing() && tlas)
        {
            uint64 tlas_rebuild_fence;
            {
                DXCommandList* cl = dx_context.get_free_compute_command_list(true);
                tlas->build(cl);
                tlas_rebuild_fence = dx_context.execute_command_list(cl);
            }

            dx_context.render_queue.wait_for_other_queue(dx_context.compute_queue, tlas_rebuild_fence);

            path_tracer.finalize_for_render();

            DXCommandList* cl = dx_context.get_free_render_command_list();

            D3D12_RESOURCE_STATES frame_result_state = D3D12_RESOURCE_STATE_COMMON;

            if (aspect_ratio_mode_changed)
            {
                cl->transition_barrier(frame_result, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_RENDER_TARGET);
                cl->clear_rtv(frame_result, 0.f, 0.f, 0.f);
                frame_result_state = D3D12_RESOURCE_STATE_RENDER_TARGET;
            }

            BarrierBatcher(cl)
                .transition(hdr_color_texture, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
                .transition(frame_result, frame_result_state, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

            {
                PROFILE_ALL(cl, "Raytracing");

                common_render_data.camera_cbv = unjittered_camera_cbv;
                path_tracer.render(cl, *tlas, hdr_color_texture, unjittered_camera, common_render_data);
            }

            cl->reset_to_dynamic_descriptor_heap();

            BarrierBatcher(cl)
                .transition(hdr_color_texture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

            tonemap(cl, hdr_color_texture, ldr_post_processing_texture, frame_settings.tonemap_settings);

            BarrierBatcher(cl)
                .transition(ldr_post_processing_texture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

            present(cl, ldr_post_processing_texture, frame_result, {0.f});

            BarrierBatcher(cl)
                .transition(hdr_color_texture, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET)
                .transition(ldr_post_processing_texture, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
                .transition(frame_result, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COMMON);

            dx_context.execute_command_list(cl);
        }

        ao_was_on_last_frame = frame_settings.enable_ao;
        sss_was_on_last_frame = frame_settings.enable_sss;
    }
}