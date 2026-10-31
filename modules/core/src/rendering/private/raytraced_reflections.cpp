// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "rendering/raytraced_reflections.h"
#include "rendering/render_resources.h"
#include "rendering/pbr_raytracer.h"

#include "core/color.h"

#include "dx/dx_profiling.h"
#include "dx/dx_barrier_batcher.h"

#include "rt_reflections_rs.hlsli"

namespace era_engine
{
    struct RTReflectionsRaytracer : PBRRaytracer
    {
        void initialize();

        void render(DXCommandList* cl, const RaytracingTLAS& tlas,
            ref<DXTexture> depth_stencil_buffer,
            ref<DXTexture> world_normals_roughness_texture,
            ref<DXTexture> screen_velocities_texture,
            ref<DXTexture> raycast_texture,
            ref<DXTexture> resolve_texture,
            ref<DXTexture> temporal_history,
            ref<DXTexture> temporal_output,
            const CommonRenderData& common);

    private:
        const uint32 max_recursion_depth = 2;

        // Only descriptors in here!
        struct InputResources
        {
            DXCPUDescriptorHandle tlas;
            DXCPUDescriptorHandle sky;
            DXCPUDescriptorHandle probe_irradiance;
            DXCPUDescriptorHandle probe_depth;
            DXCPUDescriptorHandle depth_buffer;
            DXCPUDescriptorHandle world_normals_and_roughness;
            DXCPUDescriptorHandle noise;
            DXCPUDescriptorHandle screen_velocities_texture;
        };

        struct OutputResources
        {
            DXCPUDescriptorHandle output;
        };
    };

    static RTReflectionsRaytracer rt_reflections_tracer;

    void initialize_rt_reflections_pipelines()
    {
        if (!dx_context.feature_support.raytracing())
            return;

        rt_reflections_tracer.initialize();
    }

    void raytrace_rt_reflections(DXCommandList* cl, const RaytracingTLAS& tlas,
        ref<DXTexture> depth_stencil_buffer,
        ref<DXTexture> world_normals_roughness_texture,
        ref<DXTexture> screen_velocities_texture,
        ref<DXTexture> raycast_texture,
        ref<DXTexture> resolve_texture,
        ref<DXTexture> temporal_history,
        ref<DXTexture> temporal_output,
        const CommonRenderData& common)
    {
        if (!dx_context.feature_support.raytracing())
            return;

        rt_reflections_tracer.finalize_for_render();
        rt_reflections_tracer.render(cl, tlas, depth_stencil_buffer, world_normals_roughness_texture, screen_velocities_texture,
            raycast_texture, resolve_texture, temporal_history, temporal_output, common);
    }

    void RTReflectionsRaytracer::initialize()
    {
        const wchar* shader_path = L"reflections/rt_reflections_rts.hlsl";

        const uint32 num_input_resources = sizeof(InputResources) / sizeof(DXCPUDescriptorHandle);
        const uint32 num_output_resources = sizeof(OutputResources) / sizeof(DXCPUDescriptorHandle);

        CD3DX12_DESCRIPTOR_RANGE resource_ranges[] =
            {
                // Must be input first, then output
                CD3DX12_DESCRIPTOR_RANGE(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, num_input_resources, 0),
                CD3DX12_DESCRIPTOR_RANGE(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, num_output_resources, 0),
            };

        CD3DX12_ROOT_PARAMETER global_root_parameters[] =
            {
                root_descriptor_table(arraysize(resource_ranges), resource_ranges),
                root_constants<RTReflectionsCB>(0),
                root_cbv(1), // Camera.
                root_cbv(2), // Sun.
            };

        CD3DX12_STATIC_SAMPLER_DESC global_static_samplers[] =
            {
                CD3DX12_STATIC_SAMPLER_DESC(0, D3D12_FILTER_MIN_MAG_MIP_LINEAR),
                CD3DX12_STATIC_SAMPLER_DESC(1, D3D12_FILTER_MIN_MAG_MIP_POINT, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_TEXTURE_ADDRESS_MODE_CLAMP),
            };

        D3D12_ROOT_SIGNATURE_DESC global_desc =
            {
                arraysize(global_root_parameters), global_root_parameters,
                arraysize(global_static_samplers), global_static_samplers};

        PBRRaytracer::initialize(shader_path, 3 * sizeof(float), max_recursion_depth, global_desc);

        allocate_descriptor_heap_space_for_global_resources<InputResources, OutputResources>(descriptor_heap);
    }

    void ssr_temporal(DXCommandList* cl,
        ref<DXTexture> screen_velocities_texture,
        ref<DXTexture> prev_frame_screen_velocities,
        ref<DXTexture> depth_stencil_buffer,
        ref<DXTexture> raycast_texture,
        ref<DXTexture> resolve_texture,
        ref<DXTexture> ssr_temporal_history,
        ref<DXTexture> ssr_temporal_output,
        DXDynamicConstantBuffer camera_cbv);

    void RTReflectionsRaytracer::render(DXCommandList* cl, const RaytracingTLAS& tlas,
        ref<DXTexture> depth_stencil_buffer,
        ref<DXTexture> world_normals_roughness_texture,
        ref<DXTexture> screen_velocities_texture,
        ref<DXTexture> raycast_texture,
        ref<DXTexture> resolve_texture,
        ref<DXTexture> temporal_history,
        ref<DXTexture> temporal_output,
        const CommonRenderData& common)
    {
        if (!tlas.tlas)
            return;

        {
            PROFILE_ALL(cl, "Raytrace reflections");

            InputResources in;
            in.tlas = tlas.tlas->raytracing_srv;
            in.sky = common.sky->default_srv;
            in.probe_irradiance = common.light_probe_irradiance->default_srv;
            in.probe_depth = common.light_probe_depth->default_srv;
            in.depth_buffer = depth_stencil_buffer->default_srv;
            in.world_normals_and_roughness = world_normals_roughness_texture->default_srv;
            in.noise = RenderResources::noise_texture->default_srv;
            in.screen_velocities_texture = screen_velocities_texture->default_srv;

            OutputResources out;
            out.output = resolve_texture->default_uav;

            DXGPUDescriptorHandle gpu_handle = copy_global_resources_to_descriptor_heap(in, out);

            // Fill out description
            D3D12_DISPATCH_RAYS_DESC raytrace_desc;
            fill_out_ray_tracing_render_desc(binding_table.get_buffer(), raytrace_desc,
                resolve_texture->width, resolve_texture->height, 1,
                num_ray_types, binding_table.get_number_of_hit_groups());

            RTReflectionsCB cb;
            cb.sample_sky_from_texture = !common.procedural_sky;
            cb.frame_index = (uint32)dx_context.get_frame_id();

            // Set up pipeline
            cl->set_descriptor_heap(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, descriptor_heap.descriptor_heap);

            cl->set_pipeline_state(pipeline.pipeline);
            cl->set_compute_root_signature(pipeline.root_signature);

            cl->set_compute_descriptor_table(RT_REFLECTIONS_RS_RESOURCES, gpu_handle);
            cl->set_compute_32bit_constants(RT_REFLECTIONS_RS_CB, cb);
            cl->set_compute_dynamic_constant_buffer(RT_REFLECTIONS_RS_CAMERA, common.camera_cbv);
            cl->set_compute_dynamic_constant_buffer(RT_REFLECTIONS_RS_LIGHTING, common.lighting_cbv);

            cl->raytrace(raytrace_desc);

            cl->reset_to_dynamic_descriptor_heap();
        }

        {
            BarrierBatcher(cl)
                //.uav(resolveTexture)
                .transition(resolve_texture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

            ssr_temporal(cl, screen_velocities_texture, screen_velocities_texture, depth_stencil_buffer, RenderResources::black_texture, resolve_texture, temporal_history, temporal_output, common.camera_cbv);
        }
    }
}