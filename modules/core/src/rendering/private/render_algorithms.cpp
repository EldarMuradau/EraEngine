// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "rendering/render_algorithms.h"
#include "rendering/render_resources.h"
#include "rendering/render_utils.h"

#include "core/cpu_profiling.h"

#include "dx/dx_profiling.h"
#include "dx/dx_barrier_batcher.h"

#include "post_processing_rs.hlsli"
#include "ssr_rs.hlsli"
#include "light_culling_rs.hlsli"
#include "depth_only_rs.hlsli"
#include "sky_rs.hlsli"
#include "outline_rs.hlsli"
#include "transform.hlsli"
#include "visualization_rs.hlsli"

namespace era_engine
{
    static DXPipeline texture_sky_pipeline;
    static DXPipeline procedural_sky_pipeline;

    static DXPipeline outline_drawer_pipeline;

    static DXPipeline shadow_map_copy_pipeline;

    static DXPipeline world_space_frusta_pipeline;
    static DXPipeline light_culling_pipeline;

    static DXPipeline ssr_raycast_pipeline;
    static DXPipeline ssr_resolve_pipeline;
    static DXPipeline ssr_temporal_pipeline;
    static DXPipeline ssr_median_blur_pipeline;

    static DXPipeline specular_ambient_pipeline;

    static DXPipeline hierarchical_linear_depth_pipeline;

    static DXPipeline gaussian_blur_float_pipelines[GAUSSIAN_BLUR_KERNEL_SIZE_COUNT];
    static DXPipeline gaussian_blur_float4_pipelines[GAUSSIAN_BLUR_KERNEL_SIZE_COUNT];

    static DXPipeline dilation_pipeline;
    static DXPipeline erosion_pipeline;

    static DXPipeline taa_pipeline;

    static DXPipeline blit_pipeline;

    static DXPipeline bloom_threshold_pipeline;
    static DXPipeline bloom_combine_pipeline;

    static DXPipeline hbao_pipeline;
    static DXPipeline sss_pipeline;

    static DXPipeline shadow_blur_x_pipeline;
    static DXPipeline shadow_blur_y_pipeline;

    static DXPipeline tonemap_pipeline;
    static DXPipeline present_pipeline;

    static DXPipeline depth_sobel_pipeline;

    static DXPipeline visualize_sun_shadow_cascades_pipeline;

#pragma pack(push, 1)
    struct IndirectRigidDepthPrepassCommand
    {
        D3D12_VERTEX_BUFFER_VIEW vertex_buffer_view; // 16 bytes.
        D3D12_INDEX_BUFFER_VIEW index_buffer_view;   // 16 bytes -> 32 bytes total.
        DepthOnlyTransformCB transform;              // 128 bytes -> 160 bytes total.
        uint32 object_id;                            // 4 bytes -> 164 bytes total.
        D3D12_DRAW_INDEXED_ARGUMENTS draw_arguments; // 20 bytes -> 184 bytes total.
        uint32 padding[2];                           // 8 bytes -> 192 bytes total (divisible by 16).
    };

    struct IndirectAnimatedDepthPrepassCommand
    {
        D3D12_VERTEX_BUFFER_VIEW vertex_buffer_view;    // 16 bytes.
        D3D12_INDEX_BUFFER_VIEW index_buffer_view;      // 16 bytes -> 32 bytes total.
        DepthOnlyTransformCB transform;                 // 128 bytes -> 160 bytes total.
        uint32 object_id;                               // 4 bytes -> 164 bytes total.
        D3D12_GPU_VIRTUAL_ADDRESS prev_frame_positions; // 8 bytes -> 172 bytes total.
        D3D12_DRAW_INDEXED_ARGUMENTS draw_arguments;    // 20 bytes -> 192 bytes total (divisible by 16).
    };
#pragma pack(pop)

    void load_common_shaders()
    {
        // Sky
        {
            auto& desc = CREATE_GRAPHICS_PIPELINE
                             .render_targets(sky_pass_formats, arraysize(sky_pass_formats), depth_stencil_format)
                             .depth_settings(true, false)
                             .cull_front_faces();

            texture_sky_pipeline = create_reloadable_pipeline(desc, {"sky_vs", "sky_texture_ps"});
            procedural_sky_pipeline = create_reloadable_pipeline(desc, {"sky_vs", "sky_procedural_ps"});
        }

        // Outline
        {
            auto& drawer_desc = CREATE_GRAPHICS_PIPELINE
                                    .render_targets(ldr_format, depth_stencil_format)
                                    .stencil_settings(D3D12_COMPARISON_FUNC_EQUAL,
                                        D3D12_STENCIL_OP_KEEP,
                                        D3D12_STENCIL_OP_KEEP,
                                        D3D12_STENCIL_OP_KEEP,
                                        STENCIL_FLAG_SELECTED_OBJECT, // Read only selected object bit.
                                        0)
                                    .depth_settings(false, false);

            outline_drawer_pipeline = create_reloadable_pipeline(drawer_desc, {"fullscreen_triangle_vs", "outline_ps"});
        }

        // Shadow map copy
        {
            auto& desc = CREATE_GRAPHICS_PIPELINE
                             .render_targets(0, 0, shadow_depth_format)
                             .depth_settings(true, true, D3D12_COMPARISON_FUNC_ALWAYS)
                             .culling_off();

            shadow_map_copy_pipeline = create_reloadable_pipeline(desc, {"fullscreen_triangle_vs", "shadow_map_copy_ps"});
        }

        world_space_frusta_pipeline = create_reloadable_pipeline("world_space_tiled_frusta_cs");
        light_culling_pipeline = create_reloadable_pipeline("light_culling_cs");

        ssr_raycast_pipeline = create_reloadable_pipeline("ssr_raycast_cs");
        ssr_resolve_pipeline = create_reloadable_pipeline("ssr_resolve_cs");
        ssr_temporal_pipeline = create_reloadable_pipeline("ssr_temporal_cs");
        ssr_median_blur_pipeline = create_reloadable_pipeline("ssr_median_blur_cs");

        specular_ambient_pipeline = create_reloadable_pipeline("specular_ambient_cs");

        hierarchical_linear_depth_pipeline = create_reloadable_pipeline("hierarchical_linear_depth_cs");

        gaussian_blur_float_pipelines[GAUSSIAN_BLUR_5X5] = create_reloadable_pipeline("gaussian_blur_5x5_float_cs");
        gaussian_blur_float_pipelines[GAUSSIAN_BLUR_9X9] = create_reloadable_pipeline("gaussian_blur_9x9_float_cs");
        gaussian_blur_float_pipelines[GAUSSIAN_BLUR_13X13] = create_reloadable_pipeline("gaussian_blur_13x13_float_cs");

        gaussian_blur_float4_pipelines[GAUSSIAN_BLUR_5X5] = create_reloadable_pipeline("gaussian_blur_5x5_float4_cs");
        gaussian_blur_float4_pipelines[GAUSSIAN_BLUR_9X9] = create_reloadable_pipeline("gaussian_blur_9x9_float4_cs");
        gaussian_blur_float4_pipelines[GAUSSIAN_BLUR_13X13] = create_reloadable_pipeline("gaussian_blur_13x13_float4_cs");

        dilation_pipeline = create_reloadable_pipeline("dilation_cs");
        erosion_pipeline = create_reloadable_pipeline("erosion_cs");

        taa_pipeline = create_reloadable_pipeline("taa_cs");

        blit_pipeline = create_reloadable_pipeline("blit_cs");

        bloom_threshold_pipeline = create_reloadable_pipeline("bloom_threshold_cs");
        bloom_combine_pipeline = create_reloadable_pipeline("bloom_combine_cs");

        hbao_pipeline = create_reloadable_pipeline("hbao_cs");
        sss_pipeline = create_reloadable_pipeline("sss_cs");

        shadow_blur_x_pipeline = create_reloadable_pipeline("shadow_blur_x_cs");
        shadow_blur_y_pipeline = create_reloadable_pipeline("shadow_blur_y_cs");

        tonemap_pipeline = create_reloadable_pipeline("tonemap_cs");
        present_pipeline = create_reloadable_pipeline("present_cs");

        depth_sobel_pipeline = create_reloadable_pipeline("depth_sobel_cs");

        visualize_sun_shadow_cascades_pipeline = create_reloadable_pipeline("sun_shadow_cascades_cs");
    }

    void depth_pre_pass(DXCommandList* cl,
        const DXRenderTarget& depth_only_render_target,
        const OpaqueRenderPass* opaque_render_pass,
        const mat4& view_proj, const mat4& prev_frame_view_proj,
        const CommonRenderData& common)
    {
        if (opaque_render_pass)
        {
            PROFILE_ALL(cl, "Depth pre-pass");

            cl->set_render_target(depth_only_render_target);
            cl->set_viewport(depth_only_render_target.viewport);

            PipelineSetupFunc last_setup_func = 0;

            for (const auto& dc : opaque_render_pass->depth_prepass)
            {
                if (dc.setup != last_setup_func)
                {
                    dc.setup(cl, common);
                    last_setup_func = dc.setup;
                }
                dc.render(cl, view_proj, prev_frame_view_proj, dc.data);
            }
        }
    }

    void textured_sky(DXCommandList* cl,
        const DXRenderTarget& sky_render_target,
        const mat4& proj, const mat4& view, const mat4& prev_frame_view,
        ref<DXTexture> sky,
        float sky_intensity,
        vec2 jitter, vec2 prev_frame_jitter)
    {
        PROFILE_ALL(cl, "Sky");

        cl->set_render_target(sky_render_target);
        cl->set_viewport(sky_render_target.viewport);

        cl->set_pipeline_state(*texture_sky_pipeline.pipeline);
        cl->set_graphics_root_signature(*texture_sky_pipeline.root_signature);

        cl->set_graphics_32bit_constants(SKY_RS_TRANSFORM, SkyTransformCB{proj * create_sky_view_matrix(view), proj * create_sky_view_matrix(prev_frame_view)});
        cl->set_graphics_32bit_constants(SKY_RS_CB, SkyCB{jitter, prev_frame_jitter, sky_intensity});
        cl->set_descriptor_heap_srv(SKY_RS_TEX, 0, sky);

        cl->draw_cube_triangle_strip();
    }

    void procedural_sky(DXCommandList* cl,
        const DXRenderTarget& sky_render_target,
        const mat4& proj, const mat4& view, const mat4& prev_frame_view,
        vec3 sun_direction, float sky_intensity,
        vec2 jitter, vec2 prev_frame_jitter)
    {
        PROFILE_ALL(cl, "Sky");

        cl->set_render_target(sky_render_target);
        cl->set_viewport(sky_render_target.viewport);

        cl->set_pipeline_state(*procedural_sky_pipeline.pipeline);
        cl->set_graphics_root_signature(*procedural_sky_pipeline.root_signature);

        cl->set_graphics_32bit_constants(SKY_RS_TRANSFORM, SkyTransformCB{proj * create_sky_view_matrix(view), proj * create_sky_view_matrix(prev_frame_view)});
        cl->set_graphics_32bit_constants(SKY_RS_CB, SkyCB{jitter, prev_frame_jitter, sky_intensity, sun_direction});

        cl->draw_cube_triangle_strip();
    }

    static void render_shadow(DXCommandList* cl, const DefaultRenderCommandBuffer<uint64>& pass, const mat4& view_proj, const CommonRenderData& common)
    {
        PipelineSetupFunc last_setup_func = 0;

        for (const auto& dc : pass)
        {
            if (dc.setup != last_setup_func)
            {
                dc.setup(cl, common);
                last_setup_func = dc.setup;
            }
            dc.render(cl, view_proj, dc.data);
        }
    }

    static void render_sun_shadows(DXCommandList* cl, const SunShadowRenderPass* pass, const CommonRenderData& common, bool dynamic)
    {
        for (uint32 render_cascade_index = 0; render_cascade_index < pass->num_cascades; ++render_cascade_index)
        {
            const SunCascadeRenderPass& cascade = pass->cascades[render_cascade_index];

            ShadowMapViewport vp = cascade.viewport;
            cl->set_viewport(vp.x, vp.y, vp.size, vp.size);

            for (uint32 i = 0; i <= render_cascade_index; ++i)
            {
                render_shadow(cl, (!dynamic) ? pass->cascades[i].static_pass : pass->cascades[i].dynamic_pass, cascade.view_proj, common);
            }
        }
    }

    static void render_spot_shadows(DXCommandList* cl, const SpotShadowRenderPass* pass, const CommonRenderData& common, bool dynamic)
    {
        ShadowMapViewport vp = pass->viewport;
        cl->set_viewport(vp.x, vp.y, vp.size, vp.size);

        render_shadow(cl, (!dynamic) ? pass->static_pass : pass->dynamic_pass, pass->view_proj_matrix, common);
    }

    static void render_point_shadows(DXCommandList* cl, const PointShadowRenderPass* pass, const CommonRenderData& common, bool dynamic)
    {
        for (uint32 v = 0; v < 2; ++v)
        {
            PROFILE_ALL(cl, (v == 0) ? "First hemisphere" : "Second hemisphere");

            ShadowMapViewport vp = (v == 0) ? pass->viewport0 : pass->viewport1;
            cl->set_viewport(vp.x, vp.y, vp.size, vp.size);

            PipelineSetupFunc last_setup_func = 0;

            for (const auto& dc : (!dynamic) ? pass->static_pass : pass->dynamic_pass)
            {
                if (dc.setup != last_setup_func)
                {
                    dc.setup(cl, common);
                    last_setup_func = dc.setup;

                    PointShadowCB cb =
                        {
                            pass->light_position,
                            pass->max_distance,
                            (v == 0) ? 1.f : -1.f};

                    cl->set_graphics_32bit_constants(POINT_SHADOW_RS_CB, cb);
                }
                dc.render(cl, mat4::identity, dc.data);
            }
        }
    }

    void shadow_passes(DXCommandList* cl,
        const SunShadowRenderPass** sun_shadow_render_passes, uint32 num_sun_light_shadow_render_passes,
        const SpotShadowRenderPass** spot_light_shadow_render_passes, uint32 num_spot_light_shadow_render_passes,
        const PointShadowRenderPass** point_light_shadow_render_passes, uint32 num_point_light_shadow_render_passes,
        const CommonRenderData& common)
    {
        if (num_sun_light_shadow_render_passes || num_spot_light_shadow_render_passes || num_point_light_shadow_render_passes)
        {
            PROFILE_ALL(cl, "Shadow map pass");

            ClearRect clear_rects[128];
            uint32 num_clear_rects = 0;

            ShadowMapViewport copies_from_static_cache[128];
            uint32 num_copies_from_static_cache = 0;

            ShadowMapViewport copies_to_static_cache[128];
            uint32 num_copies_to_static_cache = 0;

            {
                for (uint32 pass_index = 0; pass_index < num_sun_light_shadow_render_passes; ++pass_index)
                {
                    auto pass = sun_shadow_render_passes[pass_index];

                    for (uint32 cascade_index = 0; cascade_index < pass->num_cascades; ++cascade_index)
                    {
                        const SunCascadeRenderPass& cascade = pass->cascades[cascade_index];
                        ShadowMapViewport vp = cascade.viewport;

                        if (pass->copy_from_static_cache)
                        {
                            copies_from_static_cache[num_copies_from_static_cache++] = vp;
                        }
                        else
                        {
                            clear_rects[num_clear_rects++] = {vp.x, vp.y, vp.size, vp.size};
                            copies_to_static_cache[num_copies_to_static_cache++] = vp;
                        }
                    }
                }

                for (uint32 i = 0; i < num_spot_light_shadow_render_passes; ++i)
                {
                    ShadowMapViewport vp = spot_light_shadow_render_passes[i]->viewport;
                    if (spot_light_shadow_render_passes[i]->copy_from_static_cache)
                    {
                        copies_from_static_cache[num_copies_from_static_cache++] = vp;
                    }
                    else
                    {
                        clear_rects[num_clear_rects++] = {vp.x, vp.y, vp.size, vp.size};
                        copies_to_static_cache[num_copies_to_static_cache++] = vp;
                    }
                }

                for (uint32 i = 0; i < num_point_light_shadow_render_passes; ++i)
                {
                    for (uint32 j = 0; j < 2; ++j)
                    {
                        ShadowMapViewport vp = (j == 0) ? point_light_shadow_render_passes[i]->viewport0 : point_light_shadow_render_passes[i]->viewport1;
                        bool copy = (j == 0) ? point_light_shadow_render_passes[i]->copy_from_static_cache0 : point_light_shadow_render_passes[i]->copy_from_static_cache1;

                        if (copy)
                        {
                            copies_from_static_cache[num_copies_from_static_cache++] = vp;
                        }
                        else
                        {
                            clear_rects[num_clear_rects++] = {vp.x, vp.y, vp.size, vp.size};
                            copies_to_static_cache[num_copies_to_static_cache++] = vp;
                        }
                    }
                }
            }

            if (!enable_static_shadow_map_caching)
            {
                num_copies_to_static_cache = 0;
            }

            if (num_copies_from_static_cache)
            {
                PROFILE_ALL(cl, "Copy from static shadow map cache");
                copy_shadow_map_parts(cl, RenderResources::static_shadow_map_cache, RenderResources::shadow_map, copies_from_static_cache, num_copies_from_static_cache);
            }

            if (num_clear_rects)
            {
                cl->clear_depth(RenderResources::shadow_map->default_dsv, 1.f, clear_rects, num_clear_rects);
            }

            if (num_copies_to_static_cache)
            {
                BarrierBatcher(cl)
                    .transition_begin(RenderResources::static_shadow_map_cache, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_DEPTH_WRITE);
            }

            auto shadow_render_target = DXRenderTarget(SHADOW_MAP_WIDTH, SHADOW_MAP_HEIGHT)
                                            .depth_attachment(RenderResources::shadow_map);

            cl->set_render_target(shadow_render_target);

            {
                PROFILE_ALL(cl, "Static geometry");

                for (uint32 pass_index = 0; pass_index < num_sun_light_shadow_render_passes; ++pass_index)
                {
                    render_sun_shadows(cl, sun_shadow_render_passes[pass_index], common, false);
                }
                for (uint32 pass_index = 0; pass_index < num_spot_light_shadow_render_passes; ++pass_index)
                {
                    render_spot_shadows(cl, spot_light_shadow_render_passes[pass_index], common, false);
                }
                for (uint32 pass_index = 0; pass_index < num_point_light_shadow_render_passes; ++pass_index)
                {
                    render_point_shadows(cl, point_light_shadow_render_passes[pass_index], common, false);
                }
            }

            if (num_copies_to_static_cache)
            {
                PROFILE_ALL(cl, "Copy to static shadow map cache");

                BarrierBatcher(cl)
                    .transition(RenderResources::shadow_map, D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE)
                    .transition_end(RenderResources::static_shadow_map_cache, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_DEPTH_WRITE);

                copy_shadow_map_parts(cl, RenderResources::shadow_map, RenderResources::static_shadow_map_cache, copies_to_static_cache, num_copies_to_static_cache);

                BarrierBatcher(cl)
                    .transition(RenderResources::shadow_map, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_DEPTH_WRITE)
                    .transition(RenderResources::static_shadow_map_cache, D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
            }

            cl->set_render_target(shadow_render_target);

            {
                PROFILE_ALL(cl, "Dynamic geometry");

                for (uint32 pass_index = 0; pass_index < num_sun_light_shadow_render_passes; ++pass_index)
                {
                    render_sun_shadows(cl, sun_shadow_render_passes[pass_index], common, true);
                }
                for (uint32 pass_index = 0; pass_index < num_spot_light_shadow_render_passes; ++pass_index)
                {
                    render_spot_shadows(cl, spot_light_shadow_render_passes[pass_index], common, true);
                }
                for (uint32 pass_index = 0; pass_index < num_point_light_shadow_render_passes; ++pass_index)
                {
                    render_point_shadows(cl, point_light_shadow_render_passes[pass_index], common, true);
                }
            }
        }
    }

    void opaque_light_pass(DXCommandList* cl,
        const DXRenderTarget& render_target,
        const OpaqueRenderPass* opaque_render_pass,
        const CommonRenderData& common,
        const mat4& view_proj)
    {
        if (opaque_render_pass && opaque_render_pass->pass.size() > 0)
        {
            PROFILE_ALL(cl, "Main opaque light pass");

            cl->set_render_target(render_target);
            cl->set_viewport(render_target.viewport);

            PipelineSetupFunc last_setup_func = 0;

            for (const auto& dc : opaque_render_pass->pass)
            {
                if (dc.setup != last_setup_func)
                {
                    dc.setup(cl, common);
                    last_setup_func = dc.setup;
                }
                dc.render(cl, view_proj, dc.data);
            }
        }
    }

    void transparent_light_pass(DXCommandList* cl,
        const DXRenderTarget& render_target,
        const TransparentRenderPass* transparent_render_pass,
        const CommonRenderData& common,
        const mat4& view_proj)
    {
        if (transparent_render_pass && transparent_render_pass->pass.size() > 0)
        {
            PROFILE_ALL(cl, "Transparent light pass");

            cl->set_render_target(render_target);
            cl->set_viewport(render_target.viewport);

            PipelineSetupFunc last_setup_func = 0;

            for (const auto& dc : transparent_render_pass->pass)
            {
                if (dc.setup != last_setup_func)
                {
                    dc.setup(cl, common);
                    last_setup_func = dc.setup;
                }
                dc.render(cl, view_proj, dc.data);
            }
        }
    }

    void transparent_motion_vectors_pass(DXCommandList* cl,
        const DXRenderTarget& render_target,
        const TransparentRenderPass* transparent_render_pass,
        const mat4& view_proj, const mat4& prev_frame_view_proj,
        const CommonRenderData& common)
    {
        if (transparent_render_pass && transparent_render_pass->motion_vectors.size() > 0)
        {
            PROFILE_ALL(cl, "Transparent motion vectors");

            cl->set_render_target(render_target);
            cl->set_viewport(render_target.viewport);

            PipelineSetupFunc last_setup_func = 0;

            for (const auto& dc : transparent_render_pass->motion_vectors)
            {
                if (dc.setup != last_setup_func)
                {
                    dc.setup(cl, common);
                    last_setup_func = dc.setup;
                }
                dc.render(cl, view_proj, prev_frame_view_proj, dc.data);
            }
        }
    }

    void ldr_pass(DXCommandList* cl,
        const DXRenderTarget& ldr_render_target,
        ref<DXTexture> depth_stencil_buffer,
        const LDRRenderPass* ldr_render_pass,
        const CommonRenderData& common,
        const mat4& view_proj)
    {
        if (ldr_render_pass)
        {
            PROFILE_ALL(cl, "LDR pass");

            cl->set_render_target(ldr_render_target);
            cl->set_viewport(ldr_render_target.viewport);


            if (ldr_render_pass->ldr_pass.size())
            {
                PROFILE_ALL(cl, "LDR Objects");

                PipelineSetupFunc last_setup_func = 0;

                for (const auto& dc : ldr_render_pass->ldr_pass)
                {
                    if (dc.setup != last_setup_func)
                    {
                        dc.setup(cl, common);
                        last_setup_func = dc.setup;
                    }
                    dc.render(cl, view_proj, dc.data);
                }
            }

            if (ldr_render_pass->overlays.size())
            {
                PROFILE_ALL(cl, "3D Overlays");

                cl->clear_depth(ldr_render_target.dsv);

                PipelineSetupFunc last_setup_func = 0;

                for (const auto& dc : ldr_render_pass->overlays)
                {
                    if (dc.setup != last_setup_func)
                    {
                        dc.setup(cl, common);
                        last_setup_func = dc.setup;
                    }
                    dc.render(cl, view_proj, dc.data);
                }
            }

            if (ldr_render_pass->outlines.size())
            {
                PROFILE_ALL(cl, "Outlines");

                cl->set_primitive_topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
                cl->set_stencil_reference(STENCIL_FLAG_SELECTED_OBJECT);

                PipelineSetupFunc last_setup_func = 0;

                for (const auto& dc : ldr_render_pass->outlines)
                {
                    if (dc.setup != last_setup_func)
                    {
                        dc.setup(cl, common);
                        last_setup_func = dc.setup;
                    }
                    dc.render(cl, view_proj, dc.data);
                }

                // Draw outline
                cl->transition_barrier(depth_stencil_buffer, D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_DEPTH_READ);

                cl->set_pipeline_state(*outline_drawer_pipeline.pipeline);
                cl->set_graphics_root_signature(*outline_drawer_pipeline.root_signature);

                cl->set_graphics_32bit_constants(OUTLINE_RS_CB, OutlineDrawerCB{(int)depth_stencil_buffer->width, (int)depth_stencil_buffer->height});
                cl->set_descriptor_heap_resource(OUTLINE_RS_STENCIL, 0, 1, depth_stencil_buffer->stencil_srv);

                cl->draw_fullscreen_triangle();

                cl->transition_barrier(depth_stencil_buffer, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_DEPTH_READ, D3D12_RESOURCE_STATE_DEPTH_WRITE);
            }
        }
    }

    void copy_shadow_map_parts(DXCommandList* cl,
        ref<DXTexture> from,
        ref<DXTexture> to,
        ShadowMapViewport* copies, uint32 num_copies)
    {
        // Since copies from or to parts of a depth-stencil texture are not allowed (even though they work on at least some hardware),
        // we copy to and from the static shadow map cache via a shader, and not via CopyTextureRegion.

        auto& shadow_render_target = DXRenderTarget(to->width, to->height)
                                         .depth_attachment(to);

        cl->set_render_target(shadow_render_target);

        cl->set_pipeline_state(*shadow_map_copy_pipeline.pipeline);
        cl->set_graphics_root_signature(*shadow_map_copy_pipeline.root_signature);

        cl->set_descriptor_heap_srv(1, 0, from);

        for (uint32 i = 0; i < num_copies; ++i)
        {
            ShadowMapViewport vp = copies[i];
            cl->set_graphics_32bit_constants(0, vec4((float)vp.x / SHADOW_MAP_WIDTH, (float)vp.y / SHADOW_MAP_HEIGHT, (float)vp.size / SHADOW_MAP_WIDTH, (float)vp.size / SHADOW_MAP_HEIGHT));
            cl->set_viewport(vp.x, vp.y, vp.size, vp.size);
            cl->draw_fullscreen_triangle();
        }
    }

    void light_and_decal_culling(DXCommandList* cl,
        ref<DXTexture> depth_stencil_buffer,
        ref<DXBuffer> point_lights,
        ref<DXBuffer> spot_lights,
        ref<DXBuffer> decals,
        LightCulling culling,
        uint32 num_point_lights, uint32 num_spot_lights, uint32 num_decals,
        DXDynamicConstantBuffer camera_cbv)
    {
        // if (numPointLights || numSpotLights || numDecals)
        {
            PROFILE_ALL(cl, "Cull lights & decals");

            // Tiled frusta
            {
                PROFILE_ALL(cl, "Create world space frusta");

                cl->set_pipeline_state(*world_space_frusta_pipeline.pipeline);
                cl->set_compute_root_signature(*world_space_frusta_pipeline.root_signature);
                cl->set_compute_dynamic_constant_buffer(WORLD_SPACE_TILED_FRUSTA_RS_CAMERA, camera_cbv);
                cl->set_compute_32bit_constants(WORLD_SPACE_TILED_FRUSTA_RS_CB, FrustaCB{culling.num_culling_tiles_x, culling.num_culling_tiles_y});
                cl->set_root_compute_uav(WORLD_SPACE_TILED_FRUSTA_RS_FRUSTA_UAV, culling.tiled_world_space_frusta_buffer);
                cl->dispatch(bucketize(culling.num_culling_tiles_x, 16), bucketize(culling.num_culling_tiles_y, 16));
            }

            BarrierBatcher(cl)
                .uav(culling.tiled_world_space_frusta_buffer);

            // Culling
            {
                PROFILE_ALL(cl, "Sort objects into tiles");

                cl->set_descriptor_heap(dx_context.srv_uav_allocator.type, dx_context.srv_uav_allocator_shader_visible.get_heap(0).Get());
                cl->clear_uav(culling.tiled_culling_index_counter, 0u);
                cl->reset_to_dynamic_descriptor_heap();
                // cl->uavBarrier(tiledCullingIndexCounter);
                cl->set_pipeline_state(*light_culling_pipeline.pipeline);
                cl->set_compute_root_signature(*light_culling_pipeline.root_signature);
                cl->set_compute_dynamic_constant_buffer(LIGHT_CULLING_RS_CAMERA, camera_cbv);
                cl->set_compute_32bit_constants(LIGHT_CULLING_RS_CB, LightCullingCB{culling.num_culling_tiles_x, num_point_lights, num_spot_lights, num_decals});
                cl->set_descriptor_heap_srv(LIGHT_CULLING_RS_SRV_UAV, 0, depth_stencil_buffer);
                cl->set_descriptor_heap_srv(LIGHT_CULLING_RS_SRV_UAV, 1, culling.tiled_world_space_frusta_buffer);
                cl->set_descriptor_heap_srv(LIGHT_CULLING_RS_SRV_UAV, 2, point_lights ? point_lights->default_srv : RenderResources::null_buffer_srv);
                cl->set_descriptor_heap_srv(LIGHT_CULLING_RS_SRV_UAV, 3, spot_lights ? spot_lights->default_srv : RenderResources::null_buffer_srv);
                cl->set_descriptor_heap_srv(LIGHT_CULLING_RS_SRV_UAV, 4, decals ? decals->default_srv : RenderResources::null_buffer_srv);
                cl->set_descriptor_heap_uav(LIGHT_CULLING_RS_SRV_UAV, 5, culling.tiled_culling_grid);
                cl->set_descriptor_heap_uav(LIGHT_CULLING_RS_SRV_UAV, 6, culling.tiled_culling_index_counter);
                cl->set_descriptor_heap_uav(LIGHT_CULLING_RS_SRV_UAV, 7, culling.tiled_objects_index_list);
                cl->dispatch(culling.num_culling_tiles_x, culling.num_culling_tiles_y);
            }

            BarrierBatcher(cl)
                .uav(culling.tiled_culling_grid)
                .uav(culling.tiled_objects_index_list);
        }
    }

    void linear_depth_pyramid(DXCommandList* cl,
        ref<DXTexture> depth_stencil_buffer,
        ref<DXTexture> linear_depth_buffer,
        vec4 projection_params)
    {
        PROFILE_ALL(cl, "Linear depth pyramid");

        cl->set_pipeline_state(*hierarchical_linear_depth_pipeline.pipeline);
        cl->set_compute_root_signature(*hierarchical_linear_depth_pipeline.root_signature);

        float width = ceilf(depth_stencil_buffer->width * 0.5f);
        float height = ceilf(depth_stencil_buffer->height * 0.5f);

        cl->set_compute_32bit_constants(HIERARCHICAL_LINEAR_DEPTH_RS_CB, HierarchicalLinearDepthCB{projection_params, vec2(1.f / width, 1.f / height)});
        cl->set_descriptor_heap_uav(HIERARCHICAL_LINEAR_DEPTH_RS_TEXTURES, 0, linear_depth_buffer->uav_at(0));
        cl->set_descriptor_heap_uav(HIERARCHICAL_LINEAR_DEPTH_RS_TEXTURES, 1, linear_depth_buffer->uav_at(1));
        cl->set_descriptor_heap_uav(HIERARCHICAL_LINEAR_DEPTH_RS_TEXTURES, 2, linear_depth_buffer->uav_at(2));
        cl->set_descriptor_heap_uav(HIERARCHICAL_LINEAR_DEPTH_RS_TEXTURES, 3, linear_depth_buffer->uav_at(3));
        cl->set_descriptor_heap_uav(HIERARCHICAL_LINEAR_DEPTH_RS_TEXTURES, 4, linear_depth_buffer->uav_at(4));
        cl->set_descriptor_heap_uav(HIERARCHICAL_LINEAR_DEPTH_RS_TEXTURES, 5, linear_depth_buffer->uav_at(5));
        cl->set_descriptor_heap_srv(HIERARCHICAL_LINEAR_DEPTH_RS_TEXTURES, 6, depth_stencil_buffer);

        cl->dispatch(bucketize((uint32)width, POST_PROCESSING_BLOCK_SIZE), bucketize((uint32)height, POST_PROCESSING_BLOCK_SIZE));
    }

    void gaussian_blur(DXCommandList* cl,
        ref<DXTexture> input_output,
        ref<DXTexture> temp,
        uint32 input_mip, uint32 output_mip, GaussianBlurKernelSize kernel, uint32 num_iterations)
    {
        PROFILE_ALL(cl, "Gaussian Blur");

        ASSERT(input_output->format == temp->format);

        uint32 num_channels = get_number_of_channels(input_output->format);

        // Maybe we could add more specializations for 2 and 3 channels.
        DXPipeline& pipeline = ((num_channels == 1) ? gaussian_blur_float_pipelines : gaussian_blur_float4_pipelines)[kernel];

        cl->set_pipeline_state(*pipeline.pipeline);
        cl->set_compute_root_signature(*pipeline.root_signature);

        uint32 output_width = max(1u, input_output->width >> output_mip);
        uint32 output_height = max(1u, input_output->height >> output_mip);

        uint32 width_buckets = bucketize(output_width, POST_PROCESSING_BLOCK_SIZE);
        uint32 height_buckets = bucketize(output_height, POST_PROCESSING_BLOCK_SIZE);

        ASSERT(input_mip <= output_mip); // Currently only downsampling supported.

        float scale = 1.f / (1 << (output_mip - input_mip));

        uint32 source_mip = input_mip;
        GaussianBlurCB cb = {vec2(1.f / output_width, 1.f / output_height), scale};

        for (uint32 i = 0; i < num_iterations; ++i)
        {
            PROFILE_ALL(cl, "Iteration");

            {
                PROFILE_ALL(cl, "Vertical");

                DXCPUDescriptorHandle temp_uav = temp->uav_at(output_mip);

                // Vertical pass
                cb.direction_and_source_mip_level = (1 << 16) | source_mip;
                cl->set_compute_32bit_constants(GAUSSIAN_BLUR_RS_CB, cb);
                cl->set_descriptor_heap_uav(GAUSSIAN_BLUR_RS_TEXTURES, 0, temp_uav);
                cl->set_descriptor_heap_srv(GAUSSIAN_BLUR_RS_TEXTURES, 1, input_output);

                cl->dispatch(width_buckets, height_buckets);

                BarrierBatcher(cl)
                    //.uav(temp)
                    .transition(temp, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
                    .transition(input_output, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            }

            cb.step_scale = 1.f;
            source_mip = output_mip; // From here on we sample from the output mip

            {
                PROFILE_ALL(cl, "Horizontal");

                DXCPUDescriptorHandle output_uav = input_output->uav_at(output_mip);

                // Horizontal pass
                cb.direction_and_source_mip_level = (0 << 16) | source_mip;
                cl->set_compute_32bit_constants(GAUSSIAN_BLUR_RS_CB, cb);
                cl->set_descriptor_heap_uav(GAUSSIAN_BLUR_RS_TEXTURES, 0, output_uav);
                cl->set_descriptor_heap_srv(GAUSSIAN_BLUR_RS_TEXTURES, 1, temp);

                cl->dispatch(width_buckets, height_buckets);

                BarrierBatcher(cl)
                    //.uav(inputOutput)
                    .transition(temp, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
                    .transition(input_output, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
            }
        }
    }

    static void morphology_common(DXCommandList* cl, DXPipeline& pipeline, ref<DXTexture> input_output, ref<DXTexture> temp, uint32 radius, uint32 num_iterations)
    {
        if (radius == 0 || num_iterations == 0)
            return;

        ASSERT(input_output->width == temp->width);
        ASSERT(input_output->height == temp->height);

        radius = min(radius, (uint32)MORPHOLOGY_MAX_RADIUS);

        cl->set_pipeline_state(*pipeline.pipeline);
        cl->set_compute_root_signature(*pipeline.root_signature);

        for (uint32 i = 0; i < num_iterations; ++i)
        {
            DX_PROFILE_BLOCK(cl, "Iteration");

            {
                DX_PROFILE_BLOCK(cl, "Vertical");

                cl->set_compute_32bit_constants(MORPHOLOGY_RS_CB, MorphologyCB{radius, 1, input_output->height});
                cl->set_descriptor_heap_uav(MORPHOLOGY_RS_TEXTURES, 0, temp);
                cl->set_descriptor_heap_srv(MORPHOLOGY_RS_TEXTURES, 1, input_output);

                cl->dispatch(bucketize(input_output->height, MORPHOLOGY_BLOCK_SIZE), input_output->width);

                BarrierBatcher(cl)
                    //.uav(temp)
                    .transition(temp, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
                    .transition(input_output, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            }

            {
                DX_PROFILE_BLOCK(cl, "Horizontal");

                cl->set_compute_32bit_constants(MORPHOLOGY_RS_CB, MorphologyCB{radius, 0, input_output->width});
                cl->set_descriptor_heap_uav(MORPHOLOGY_RS_TEXTURES, 0, input_output);
                cl->set_descriptor_heap_srv(MORPHOLOGY_RS_TEXTURES, 1, temp);

                cl->dispatch(bucketize(input_output->width, MORPHOLOGY_BLOCK_SIZE), input_output->height);

                BarrierBatcher(cl)
                    //.uav(inputOutput)
                    .transition(temp, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
                    .transition(input_output, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
            }
        }
    }

    void dilate(DXCommandList* cl, ref<DXTexture> input_output, ref<DXTexture> temp, uint32 radius, uint32 num_iterations)
    {
        DX_PROFILE_BLOCK(cl, "Dilate");
        morphology_common(cl, dilation_pipeline, input_output, temp, radius, num_iterations);
    }

    void erode(DXCommandList* cl, ref<DXTexture> input_output, ref<DXTexture> temp, uint32 radius, uint32 num_iterations)
    {
        DX_PROFILE_BLOCK(cl, "Erode");
        morphology_common(cl, erosion_pipeline, input_output, temp, radius, num_iterations);
    }

    void depth_sobel(DXCommandList* cl, ref<DXTexture> input, ref<DXTexture> output, vec4 projection_params, float threshold)
    {
        DX_PROFILE_BLOCK(cl, "Depth sobel");

        cl->set_pipeline_state(*depth_sobel_pipeline.pipeline);
        cl->set_compute_root_signature(*depth_sobel_pipeline.root_signature);

        cl->set_compute_32bit_constants(DEPTH_SOBEL_RS_CB, DepthSobelCB{projection_params, threshold});
        cl->set_descriptor_heap_uav(DEPTH_SOBEL_RS_TEXTURES, 0, output);
        cl->set_descriptor_heap_srv(DEPTH_SOBEL_RS_TEXTURES, 1, input);

        cl->dispatch(bucketize(input->width, POST_PROCESSING_BLOCK_SIZE), bucketize(input->height, POST_PROCESSING_BLOCK_SIZE));
    }

    void ssr_temporal(DXCommandList* cl,
        ref<DXTexture> screen_velocities_texture,
        ref<DXTexture> prev_frame_screen_velocities,
        ref<DXTexture> depth_stencil_buffer,
        ref<DXTexture> raycast_texture,
        ref<DXTexture> resolve_texture,
        ref<DXTexture> ssr_temporal_history,
        ref<DXTexture> ssr_temporal_output,
        DXDynamicConstantBuffer camera_cbv)
    {
        uint32 resolve_width = resolve_texture->width;
        uint32 resolve_height = resolve_texture->height;

        {
            PROFILE_ALL(cl, "Temporal");

            cl->set_pipeline_state(*ssr_temporal_pipeline.pipeline);
            cl->set_compute_root_signature(*ssr_temporal_pipeline.root_signature);

            cl->set_compute_32bit_constants(SSR_TEMPORAL_RS_CB, SSRTemporalCB{vec2(1.f / resolve_width, 1.f / resolve_height)});
            cl->set_compute_dynamic_constant_buffer(SSR_TEMPORAL_RS_CAMERA, camera_cbv);

            cl->set_descriptor_heap_uav(SSR_TEMPORAL_RS_TEXTURES, 0, ssr_temporal_output);
            cl->set_descriptor_heap_srv(SSR_TEMPORAL_RS_TEXTURES, 1, resolve_texture);
            cl->set_descriptor_heap_srv(SSR_TEMPORAL_RS_TEXTURES, 2, ssr_temporal_history);
            cl->set_descriptor_heap_srv(SSR_TEMPORAL_RS_TEXTURES, 3, screen_velocities_texture);
            cl->set_descriptor_heap_srv(SSR_TEMPORAL_RS_TEXTURES, 4, depth_stencil_buffer);
            cl->set_descriptor_heap_srv(SSR_TEMPORAL_RS_TEXTURES, 5, raycast_texture);
            cl->set_descriptor_heap_srv(SSR_TEMPORAL_RS_TEXTURES, 6, prev_frame_screen_velocities);

            cl->dispatch(bucketize(resolve_width, SSR_BLOCK_SIZE), bucketize(resolve_height, SSR_BLOCK_SIZE));

            BarrierBatcher(cl)
                //.uav(ssrOutput)
                .transition(ssr_temporal_output, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
                .transition(ssr_temporal_history, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
                .transition(resolve_texture, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        }

        {
            PROFILE_ALL(cl, "Median Blur");

            cl->set_pipeline_state(*ssr_median_blur_pipeline.pipeline);
            cl->set_compute_root_signature(*ssr_median_blur_pipeline.root_signature);

            cl->set_compute_32bit_constants(SSR_MEDIAN_BLUR_RS_CB, SSRMedianBlurCB{vec2(1.f / resolve_width, 1.f / resolve_height)});

            cl->set_descriptor_heap_uav(SSR_MEDIAN_BLUR_RS_TEXTURES, 0, resolve_texture); // We reuse the resolve texture here.
            cl->set_descriptor_heap_srv(SSR_MEDIAN_BLUR_RS_TEXTURES, 1, ssr_temporal_output);

            cl->dispatch(bucketize(resolve_width, SSR_BLOCK_SIZE), bucketize(resolve_height, SSR_BLOCK_SIZE));

            BarrierBatcher(cl)
                //.uav(resolveTexture)
                .transition(resolve_texture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_GENERIC_READ);
        }
    }

    void screen_space_reflections(DXCommandList* cl,
        ref<DXTexture> prev_frame_hdr,
        ref<DXTexture> depth_stencil_buffer,
        ref<DXTexture> linear_depth_buffer,
        ref<DXTexture> world_normals_roughness_texture,
        ref<DXTexture> screen_velocities_texture,
        ref<DXTexture> prev_frame_screen_velocities,
        ref<DXTexture> raycast_texture,
        ref<DXTexture> resolve_texture,
        ref<DXTexture> ssr_temporal_history,
        ref<DXTexture> ssr_temporal_output,
        SSRSettings settings,
        DXDynamicConstantBuffer camera_cbv)
    {
        PROFILE_ALL(cl, "Screen space reflections");

        uint32 raycast_width = raycast_texture->width;
        uint32 raycast_height = raycast_texture->height;

        uint32 resolve_width = resolve_texture->width;
        uint32 resolve_height = resolve_texture->height;

        {
            PROFILE_ALL(cl, "Raycast");

            cl->set_pipeline_state(*ssr_raycast_pipeline.pipeline);
            cl->set_compute_root_signature(*ssr_raycast_pipeline.root_signature);

            SSRRaycastCB raycast_settings;
            raycast_settings.num_steps = settings.num_steps;
            raycast_settings.max_distance = settings.max_distance;
            raycast_settings.stride_cutoff = settings.stride_cutoff;
            raycast_settings.min_stride = settings.min_stride;
            raycast_settings.max_stride = settings.max_stride;
            raycast_settings.dimensions = vec2((float)raycast_width, (float)raycast_height);
            raycast_settings.inv_dimensions = vec2(1.f / raycast_width, 1.f / raycast_height);
            raycast_settings.frame_index = (uint32)dx_context.get_frame_id();

            cl->set_compute_32bit_constants(SSR_RAYCAST_RS_CB, raycast_settings);
            cl->set_compute_dynamic_constant_buffer(SSR_RAYCAST_RS_CAMERA, camera_cbv);
            cl->set_descriptor_heap_uav(SSR_RAYCAST_RS_TEXTURES, 0, raycast_texture);
            cl->set_descriptor_heap_srv(SSR_RAYCAST_RS_TEXTURES, 1, depth_stencil_buffer);
            cl->set_descriptor_heap_srv(SSR_RAYCAST_RS_TEXTURES, 2, linear_depth_buffer);
            cl->set_descriptor_heap_srv(SSR_RAYCAST_RS_TEXTURES, 3, world_normals_roughness_texture);
            cl->set_descriptor_heap_srv(SSR_RAYCAST_RS_TEXTURES, 4, RenderResources::noise_texture);
            cl->set_descriptor_heap_srv(SSR_RAYCAST_RS_TEXTURES, 5, screen_velocities_texture);
            cl->set_descriptor_heap_srv(SSR_RAYCAST_RS_TEXTURES, 6, prev_frame_screen_velocities);

            cl->dispatch(bucketize(raycast_width, SSR_BLOCK_SIZE), bucketize(raycast_height, SSR_BLOCK_SIZE));

            BarrierBatcher(cl)
                //.uav(raycastTexture)
                .transition(raycast_texture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        }

        {
            PROFILE_ALL(cl, "Resolve");

            cl->set_pipeline_state(*ssr_resolve_pipeline.pipeline);
            cl->set_compute_root_signature(*ssr_resolve_pipeline.root_signature);

            cl->set_compute_32bit_constants(SSR_RESOLVE_RS_CB, SSRResolveCB{vec2((float)resolve_width, (float)resolve_height), vec2(1.f / resolve_width, 1.f / resolve_height)});
            cl->set_compute_dynamic_constant_buffer(SSR_RESOLVE_RS_CAMERA, camera_cbv);

            cl->set_descriptor_heap_uav(SSR_RESOLVE_RS_TEXTURES, 0, resolve_texture);
            cl->set_descriptor_heap_srv(SSR_RESOLVE_RS_TEXTURES, 1, depth_stencil_buffer);
            cl->set_descriptor_heap_srv(SSR_RESOLVE_RS_TEXTURES, 2, world_normals_roughness_texture);
            cl->set_descriptor_heap_srv(SSR_RESOLVE_RS_TEXTURES, 3, raycast_texture);
            cl->set_descriptor_heap_srv(SSR_RESOLVE_RS_TEXTURES, 4, prev_frame_hdr);
            cl->set_descriptor_heap_srv(SSR_RESOLVE_RS_TEXTURES, 5, screen_velocities_texture);
            cl->set_descriptor_heap_srv(SSR_RESOLVE_RS_TEXTURES, 6, prev_frame_screen_velocities);

            cl->dispatch(bucketize(resolve_width, SSR_BLOCK_SIZE), bucketize(resolve_height, SSR_BLOCK_SIZE));

            BarrierBatcher(cl)
                //.uav(resolveTexture)
                .transition(resolve_texture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        }

        ssr_temporal(cl, screen_velocities_texture, prev_frame_screen_velocities, depth_stencil_buffer, raycast_texture, resolve_texture, ssr_temporal_history, ssr_temporal_output, camera_cbv);

        BarrierBatcher(cl)
            .transition(raycast_texture, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    }

    void specular_ambient(DXCommandList* cl,
        ref<DXTexture> hdr_input,
        ref<DXTexture> ssr,
        ref<DXTexture> world_normals_texture,
        ref<DXTexture> reflectance_texture,
        ref<DXTexture> environment,
        ref<DXTexture> ao,
        ref<DXTexture> output,
        DXDynamicConstantBuffer camera_cbv)
    {
        PROFILE_ALL(cl, "Specular ambient");

        cl->set_pipeline_state(*specular_ambient_pipeline.pipeline);
        cl->set_compute_root_signature(*specular_ambient_pipeline.root_signature);

        cl->set_compute_32bit_constants(SPECULAR_AMBIENT_RS_CB, SpecularAmbientCB{vec2(1.f / output->width, 1.f / output->height)});
        cl->set_compute_dynamic_constant_buffer(SPECULAR_AMBIENT_RS_CAMERA, camera_cbv);

        cl->set_descriptor_heap_uav(SPECULAR_AMBIENT_RS_TEXTURES, 0, output);
        cl->set_descriptor_heap_srv(SPECULAR_AMBIENT_RS_TEXTURES, 1, hdr_input);
        cl->set_descriptor_heap_srv(SPECULAR_AMBIENT_RS_TEXTURES, 2, world_normals_texture);
        cl->set_descriptor_heap_srv(SPECULAR_AMBIENT_RS_TEXTURES, 3, reflectance_texture);
        cl->set_descriptor_heap_srv(SPECULAR_AMBIENT_RS_TEXTURES, 4, ssr ? ssr->default_srv : RenderResources::null_texture_srv);
        cl->set_descriptor_heap_srv(SPECULAR_AMBIENT_RS_TEXTURES, 5, environment ? environment : RenderResources::black_cube_texture);
        cl->set_descriptor_heap_srv(SPECULAR_AMBIENT_RS_TEXTURES, 6, RenderResources::brdf_tex);
        cl->set_descriptor_heap_srv(SPECULAR_AMBIENT_RS_TEXTURES, 7, ao ? ao : RenderResources::white_texture);

        cl->dispatch(bucketize(output->width, POST_PROCESSING_BLOCK_SIZE), bucketize(output->height, POST_PROCESSING_BLOCK_SIZE));
    }


    void temporal_anti_aliasing(DXCommandList* cl,
        ref<DXTexture> hdr_input,
        ref<DXTexture> screen_velocities_texture,
        ref<DXTexture> prev_frame_screen_velocities,
        ref<DXTexture> depth_stencil_buffer,
        ref<DXTexture> history,
        ref<DXTexture> output,
        vec4 jittered_camera_projection_params)
    {
        PROFILE_ALL(cl, "Temporal anti-aliasing");

        cl->set_pipeline_state(*taa_pipeline.pipeline);
        cl->set_compute_root_signature(*taa_pipeline.root_signature);

        cl->set_descriptor_heap_uav(TAA_RS_TEXTURES, 0, output);
        cl->set_descriptor_heap_srv(TAA_RS_TEXTURES, 1, hdr_input);
        cl->set_descriptor_heap_srv(TAA_RS_TEXTURES, 2, history);
        cl->set_descriptor_heap_srv(TAA_RS_TEXTURES, 3, screen_velocities_texture);
        cl->set_descriptor_heap_srv(TAA_RS_TEXTURES, 4, depth_stencil_buffer);
        cl->set_descriptor_heap_srv(TAA_RS_TEXTURES, 5, prev_frame_screen_velocities);

        uint32 render_width = depth_stencil_buffer->width;
        uint32 render_height = depth_stencil_buffer->height;
        cl->set_compute_32bit_constants(TAA_RS_CB, TAACB{jittered_camera_projection_params, vec2((float)render_width, (float)render_height)});

        cl->dispatch(bucketize(render_width, POST_PROCESSING_BLOCK_SIZE), bucketize(render_height, POST_PROCESSING_BLOCK_SIZE));

        BarrierBatcher(cl)
            //.uav(taaTextures[taaOutputIndex])
            .transition(output, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)   // Will be read by rest of post processing stack. Can stay in read state, since it is read as history next frame.
            .transition(history, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS); // Will be used as UAV next frame.
    }

    void downsample(DXCommandList* cl,
        ref<DXTexture> input,
        ref<DXTexture> output,
        ref<DXTexture> temp)
    {
        PROFILE_ALL(cl, "Downsample");

        BarrierBatcher(cl)
            .transition(output, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

        blit(cl, input, output);

        BarrierBatcher(cl)
            //.uav(prevFrameHDRColorTexture)
            .transition(output, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

        for (uint32 i = 0; i < output->num_mip_levels - 1; ++i)
        {
            gaussian_blur(cl, output, temp, i, i + 1, GAUSSIAN_BLUR_5X5);
        }
    }

    void bloom(DXCommandList* cl,
        ref<DXTexture> hdr_input,
        ref<DXTexture> output,
        ref<DXTexture> bloom_texture,
        ref<DXTexture> bloom_temp_texture,
        BloomSettings settings)
    {
        PROFILE_ALL(cl, "Bloom");

        {
            PROFILE_ALL(cl, "Threshold");

            cl->set_pipeline_state(*bloom_threshold_pipeline.pipeline);
            cl->set_compute_root_signature(*bloom_threshold_pipeline.root_signature);

            cl->set_descriptor_heap_uav(BLOOM_THRESHOLD_RS_TEXTURES, 0, bloom_texture);
            cl->set_descriptor_heap_srv(BLOOM_THRESHOLD_RS_TEXTURES, 1, hdr_input);

            cl->set_compute_32bit_constants(BLOOM_THRESHOLD_RS_CB, BloomThresholdCB{vec2(1.f / bloom_texture->width, 1.f / bloom_texture->height), settings.threshold});

            cl->dispatch(bucketize(bloom_texture->width, POST_PROCESSING_BLOCK_SIZE), bucketize(bloom_texture->height, POST_PROCESSING_BLOCK_SIZE));
        }

        BarrierBatcher(cl)
            .transition(bloom_texture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

        for (uint32 i = 0; i < bloom_texture->num_mip_levels - 1; ++i)
        {
            gaussian_blur(cl, bloom_texture, bloom_temp_texture, i, i + 1, GAUSSIAN_BLUR_9X9);
        }

        {
            PROFILE_ALL(cl, "Combine");

            cl->set_pipeline_state(*bloom_combine_pipeline.pipeline);
            cl->set_compute_root_signature(*bloom_combine_pipeline.root_signature);

            cl->set_descriptor_heap_uav(BLOOM_COMBINE_RS_TEXTURES, 0, output);
            cl->set_descriptor_heap_srv(BLOOM_COMBINE_RS_TEXTURES, 1, hdr_input);
            cl->set_descriptor_heap_srv(BLOOM_COMBINE_RS_TEXTURES, 2, bloom_texture);

            cl->set_compute_32bit_constants(BLOOM_COMBINE_RS_CB, BloomCombineCB{vec2(1.f / hdr_input->width, 1.f / hdr_input->height), settings.strength});

            cl->dispatch(bucketize(hdr_input->width, POST_PROCESSING_BLOCK_SIZE), bucketize(hdr_input->height, POST_PROCESSING_BLOCK_SIZE));
        }

        BarrierBatcher(cl)
            //.uav(bloomResult)
            .transition(output, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)         // Will be read by rest of post processing stack.
            .transition(bloom_texture, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS); // For next frame.
    }

    static void bilateral_blur_shadows(DXCommandList* cl,
        ref<DXTexture> input,
        ref<DXTexture> temp_texture,
        ref<DXTexture> linear_depth,
        ref<DXTexture> screen_velocities_texture,
        ref<DXTexture> history,
        ref<DXTexture> output)
    {
        PROFILE_ALL(cl, "Bilateral blur");

        {
            PROFILE_ALL(cl, "Horizontal");

            ShadowBlurCB cb;
            cb.dimensions.x = (float)temp_texture->width;
            cb.dimensions.y = (float)temp_texture->height;
            cb.inv_dimensions.x = 1.f / cb.dimensions.x;
            cb.inv_dimensions.y = 1.f / cb.dimensions.y;

            cl->set_pipeline_state(*shadow_blur_x_pipeline.pipeline);
            cl->set_compute_root_signature(*shadow_blur_x_pipeline.root_signature);

            cl->set_descriptor_heap_uav(SHADOW_BLUR_RS_TEXTURES, 0, temp_texture);
            cl->set_descriptor_heap_srv(SHADOW_BLUR_RS_TEXTURES, 1, input);
            cl->set_descriptor_heap_srv(SHADOW_BLUR_RS_TEXTURES, 2, linear_depth);

            cl->set_compute_32bit_constants(SHADOW_BLUR_RS_CB, cb);

            cl->dispatch(bucketize(temp_texture->width, POST_PROCESSING_BLOCK_SIZE), bucketize(temp_texture->height, POST_PROCESSING_BLOCK_SIZE));

            BarrierBatcher(cl)
                //.uav(tempTexture)
                .transition(temp_texture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
                .transition_begin(input, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        }

        {
            PROFILE_ALL(cl, "Vertical");

            ShadowBlurCB cb;
            cb.dimensions.x = (float)output->width;
            cb.dimensions.y = (float)output->height;
            cb.inv_dimensions.x = 1.f / cb.dimensions.x;
            cb.inv_dimensions.y = 1.f / cb.dimensions.y;

            cl->set_pipeline_state(*shadow_blur_y_pipeline.pipeline);
            cl->set_compute_root_signature(*shadow_blur_y_pipeline.root_signature);

            cl->set_descriptor_heap_uav(SHADOW_BLUR_RS_TEXTURES, 0, output);
            cl->set_descriptor_heap_srv(SHADOW_BLUR_RS_TEXTURES, 1, temp_texture);
            cl->set_descriptor_heap_srv(SHADOW_BLUR_RS_TEXTURES, 2, linear_depth);
            cl->set_descriptor_heap_srv(SHADOW_BLUR_RS_TEXTURES, 3, screen_velocities_texture);
            cl->set_descriptor_heap_srv(SHADOW_BLUR_RS_TEXTURES, 4, history);

            cl->set_compute_32bit_constants(SHADOW_BLUR_RS_CB, cb);

            cl->dispatch(bucketize(output->width, POST_PROCESSING_BLOCK_SIZE), bucketize(output->height, POST_PROCESSING_BLOCK_SIZE));

            BarrierBatcher(cl)
                .uav(output)
                .transition(temp_texture, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
                .transition_end(input, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        }
    }

    void ambient_occlusion(DXCommandList* cl,
        ref<DXTexture> linear_depth,
        ref<DXTexture> screen_velocities_texture,
        ref<DXTexture> ao_calculation_texture,
        ref<DXTexture> ao_blur_temp_texture,
        ref<DXTexture> history,
        ref<DXTexture> output,
        HBAOSettings settings,
        DXDynamicConstantBuffer camera_cbv)
    {
        PROFILE_ALL(cl, "HBAO");

        {
            PROFILE_ALL(cl, "Calculate AO");

            HBAOCB cb;
            cb.screen_width = ao_calculation_texture->width;
            cb.screen_height = ao_calculation_texture->height;
            cb.depth_buffer_mip_level = log2((float)linear_depth->width / ao_calculation_texture->width);
            cb.num_rays = settings.num_rays;
            cb.max_num_steps_per_ray = settings.max_num_steps_per_ray;
            cb.strength = settings.strength;
            cb.radius = settings.radius;
            cb.seed = (float)(dx_context.get_frame_id() & 0xFFFFFFFF);

            float alpha = M_TAU / settings.num_rays;
            cb.ray_delta_rotation = vec2(cos(alpha), sin(alpha));

            cl->set_pipeline_state(*hbao_pipeline.pipeline);
            cl->set_compute_root_signature(*hbao_pipeline.root_signature);

            cl->set_descriptor_heap_uav(HBAO_RS_TEXTURES, 0, ao_calculation_texture);
            cl->set_descriptor_heap_srv(HBAO_RS_TEXTURES, 1, linear_depth);

            cl->set_compute_32bit_constants(HBAO_RS_CB, cb);
            cl->set_compute_dynamic_constant_buffer(HBAO_RS_CAMERA, camera_cbv);

            cl->dispatch(bucketize(ao_calculation_texture->width, POST_PROCESSING_BLOCK_SIZE), bucketize(ao_calculation_texture->height, POST_PROCESSING_BLOCK_SIZE));

            BarrierBatcher(cl)
                //.uav(aoCalculationTexture)
                .transition(ao_calculation_texture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        }

        bilateral_blur_shadows(cl, ao_calculation_texture, ao_blur_temp_texture, linear_depth, screen_velocities_texture, history, output);
    }

    void screen_space_shadows(DXCommandList* cl,
        ref<DXTexture> linear_depth,
        ref<DXTexture> screen_velocities_texture,
        ref<DXTexture> sss_calculation_texture,
        ref<DXTexture> sss_blur_temp_texture,
        ref<DXTexture> history,
        ref<DXTexture> output,
        vec3 sun_direction,
        SSSSettings settings,
        const mat4& view,
        DXDynamicConstantBuffer camera_cbv)
    {
        PROFILE_ALL(cl, "Screen space shadows");

        {
            PROFILE_ALL(cl, "Calculate SSS");

            cl->set_pipeline_state(*sss_pipeline.pipeline);
            cl->set_compute_root_signature(*sss_pipeline.root_signature);

            SSSCB cb;
            cb.inv_dimensions = vec2(1.f / (float)sss_calculation_texture->width, 1.f / (float)sss_calculation_texture->height);
            cb.light_direction = -normalize(transform_direction(view, sun_direction));
            cb.ray_distance = settings.ray_distance;
            cb.num_steps = settings.num_steps;
            cb.thickness = settings.thickness;
            cb.max_distance_from_camera = settings.max_distance_from_camera;
            cb.distance_fadeout_range = settings.distance_fadeout_range;
            cb.inv_border_fadeout = 1.f / settings.border_fadeout;
            cb.seed = (float)(dx_context.get_frame_id() & 0xFFFFFFFF);

            cl->set_compute_32bit_constants(SSS_RS_CB, cb);
            cl->set_compute_dynamic_constant_buffer(SSS_RS_CAMERA, camera_cbv);
            cl->set_descriptor_heap_uav(SSS_RS_TEXTURES, 0, sss_calculation_texture);
            cl->set_descriptor_heap_srv(SSS_RS_TEXTURES, 1, linear_depth);

            cl->dispatch(bucketize(sss_calculation_texture->width, POST_PROCESSING_BLOCK_SIZE), bucketize(sss_calculation_texture->height, POST_PROCESSING_BLOCK_SIZE));

            BarrierBatcher(cl)
                //.uav(sssCalculationTexture)
                .transition(sss_calculation_texture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        }

        bilateral_blur_shadows(cl, sss_calculation_texture, sss_blur_temp_texture, linear_depth, screen_velocities_texture, history, output);
    }

    void tonemap(DXCommandList* cl,
        ref<DXTexture> hdr_input,
        ref<DXTexture> ldr_output,
        const TonemapSettings& settings)
    {
        PROFILE_ALL(cl, "Tonemapping");

        TonemapCB cb;
        cb.A = settings.A;
        cb.B = settings.B;
        cb.C = settings.C;
        cb.D = settings.D;
        cb.E = settings.E;
        cb.F = settings.F;
        cb.inv_evaluated_linear_white = 1.f / settings.evaluate(settings.linear_white);
        cb.exp_exposure = exp2(settings.exposure);

        cl->set_pipeline_state(*tonemap_pipeline.pipeline);
        cl->set_compute_root_signature(*tonemap_pipeline.root_signature);

        cl->set_descriptor_heap_uav(TONEMAP_RS_TEXTURES, 0, ldr_output);
        cl->set_descriptor_heap_srv(TONEMAP_RS_TEXTURES, 1, hdr_input);
        cl->set_compute_32bit_constants(TONEMAP_RS_CB, cb);

        cl->dispatch(bucketize(ldr_output->width, POST_PROCESSING_BLOCK_SIZE), bucketize(ldr_output->height, POST_PROCESSING_BLOCK_SIZE));
    }

    void blit(DXCommandList* cl, ref<DXTexture> input, ref<DXTexture> output)
    {
        DX_PROFILE_BLOCK(cl, "Blit");

        cl->set_pipeline_state(*blit_pipeline.pipeline);
        cl->set_compute_root_signature(*blit_pipeline.root_signature);

        cl->set_compute_32bit_constants(BLIT_RS_CB, BlitCB{vec2(1.f / output->width, 1.f / output->height)});
        cl->set_descriptor_heap_uav(BLIT_RS_TEXTURES, 0, output);
        cl->set_descriptor_heap_srv(BLIT_RS_TEXTURES, 1, input);

        cl->dispatch(bucketize(output->width, POST_PROCESSING_BLOCK_SIZE), bucketize(output->height, POST_PROCESSING_BLOCK_SIZE));
    }

    void present(DXCommandList* cl,
        ref<DXTexture> ldr_input,
        ref<DXTexture> output,
        SharpenSettings sharpen_settings)
    {
        PROFILE_ALL(cl, "Present");

        uint32 x_offset = (output->width - ldr_input->width) / 2;
        uint32 y_offset = (output->height - ldr_input->height) / 2;

        cl->set_pipeline_state(*present_pipeline.pipeline);
        cl->set_compute_root_signature(*present_pipeline.root_signature);

        cl->set_descriptor_heap_uav(PRESENT_RS_TEXTURES, 0, output);
        cl->set_descriptor_heap_srv(PRESENT_RS_TEXTURES, 1, ldr_input);
        cl->set_compute_32bit_constants(PRESENT_RS_CB, PresentCB{present_sdr, 0.f, sharpen_settings.strength, (x_offset << 16) | y_offset});

        cl->dispatch(bucketize(output->width, POST_PROCESSING_BLOCK_SIZE), bucketize(output->height, POST_PROCESSING_BLOCK_SIZE));
    }

    void visualize_sun_shadow_cascades(DXCommandList* cl,
        ref<DXTexture> depth_buffer,
        ref<DXTexture> output,
        DXDynamicConstantBuffer sun_cbv,
        const mat4& inv_view_proj,
        vec3 camera_position,
        vec3 camera_forward)
    {
        PROFILE_ALL(cl, "Visualize sun shadow cascades");

        cl->set_pipeline_state(*visualize_sun_shadow_cascades_pipeline.pipeline);
        cl->set_compute_root_signature(*visualize_sun_shadow_cascades_pipeline.root_signature);

        VisualizeSunShadowCascadesCB cb;
        cb.inv_view_proj = inv_view_proj;
        cb.camera_position = vec4(camera_position, 1.f);
        cb.camera_forward = vec4(camera_forward, 0.f);

        cl->set_descriptor_heap_uav(VISUALIZE_SUN_SHADOW_CASCADES_RS_TEXTURES, 0, output);
        cl->set_descriptor_heap_srv(VISUALIZE_SUN_SHADOW_CASCADES_RS_TEXTURES, 1, depth_buffer);
        cl->set_compute_dynamic_constant_buffer(VISUALIZE_SUN_SHADOW_CASCADES_RS_SUN, sun_cbv);
        cl->set_compute_32bit_constants(VISUALIZE_SUN_SHADOW_CASCADES_RS_CB, cb);

        cl->dispatch(bucketize(output->width, 16), bucketize(output->height, 16));
    }

    void LightCulling::allocate_if_necessary(uint32 render_width, uint32 render_height)
    {
        uint32 old_num_culling_tiles_x = num_culling_tiles_x;
        uint32 old_num_culling_tiles_y = num_culling_tiles_y;

        num_culling_tiles_x = bucketize(render_width, LIGHT_CULLING_TILE_SIZE);
        num_culling_tiles_y = bucketize(render_height, LIGHT_CULLING_TILE_SIZE);

        bool first_allocation = tiled_culling_grid == nullptr;

        if (first_allocation)
        {
            tiled_culling_grid = create_texture(0, num_culling_tiles_x, num_culling_tiles_y,
                DXGI_FORMAT_R32G32B32A32_UINT, false, false, true);

            tiled_culling_index_counter = create_buffer(sizeof(uint32), 1, 0, true, true);
            tiled_objects_index_list = create_buffer(sizeof(uint32),
                num_culling_tiles_x * num_culling_tiles_y * MAX_NUM_INDICES_PER_TILE * 2, 0, true);
            tiled_world_space_frusta_buffer = create_buffer(sizeof(LightCullingViewFrustum), num_culling_tiles_x * num_culling_tiles_y, 0, true);

            SET_NAME(tiled_culling_grid->resource, "Tiled culling grid");
            SET_NAME(tiled_culling_index_counter->resource, "Tiled index counter");
            SET_NAME(tiled_objects_index_list->resource, "Tiled index list");
            SET_NAME(tiled_world_space_frusta_buffer->resource, "Tiled frusta");
        }
        else if (num_culling_tiles_x != old_num_culling_tiles_x || num_culling_tiles_y != old_num_culling_tiles_y)
        {
            resize_texture(tiled_culling_grid, num_culling_tiles_x, num_culling_tiles_y);
            resize_buffer(tiled_objects_index_list, num_culling_tiles_x * num_culling_tiles_y * MAX_NUM_INDICES_PER_TILE * 2);
            resize_buffer(tiled_world_space_frusta_buffer, num_culling_tiles_x * num_culling_tiles_y);
        }
    }

}