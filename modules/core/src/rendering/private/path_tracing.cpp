// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "rendering/path_tracing.h"

#include "core/color.h"

#include "raytracing.hlsli"

#define PATH_TRACING_RS_RESOURCES 0
#define PATH_TRACING_RS_CAMERA 1
#define PATH_TRACING_RS_CB 2

namespace era_engine
{
    void PathTracer::initialize()
    {
        const wchar* shader_path = L"raytracing/path_tracing_rts.hlsl";

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
                root_cbv(0), // Camera
                root_constants<PathTracingCB>(1),
            };

        CD3DX12_STATIC_SAMPLER_DESC global_static_sampler(0, D3D12_FILTER_MIN_MAG_MIP_LINEAR);

        D3D12_ROOT_SIGNATURE_DESC global_desc =
            {
                arraysize(global_root_parameters), global_root_parameters,
                1, &global_static_sampler};

        PBRRaytracer::initialize(shader_path, settings.max_payload_size, settings.max_recursion_depth, global_desc);

        allocate_descriptor_heap_space_for_global_resources<InputResources, OutputResources>(descriptor_heap);

        old_settings = settings;
    }

    bool PathTracer::is_accumulation_valid(const RaytracingTLAS& tlas, const ref<DXTexture>& output, const CameraCB& camera) const
    {
        return last_frame_id + 1 == dx_context.get_frame_id() && last_width == output->width && last_height == output->height && memcmp(&last_view_proj, &camera.view_proj, sizeof(mat4)) == 0 && last_instances.size() == tlas.all_instances.size() && memcmp(last_instances.data(), tlas.all_instances.data(), sizeof(D3D12_RAYTRACING_INSTANCE_DESC) * last_instances.size()) == 0 && memcmp(&settings, &old_settings, sizeof(PathTracerSettings)) == 0;
    }

    void PathTracer::render(DXCommandList* cl, const RaytracingTLAS& tlas,
        const ref<DXTexture>& output,
        const CameraCB& camera,
        const CommonRenderData& common)
    {
        if (!is_accumulation_valid(tlas, output, camera))
        {
            reset_rendering();

            old_settings = settings;
            last_width = output->width;
            last_height = output->height;
            last_view_proj = camera.view_proj;
            last_instances = tlas.all_instances;
        }
        last_frame_id = dx_context.get_frame_id();

        InputResources in;
        in.tlas = tlas.tlas->raytracing_srv;
        in.sky = common.sky->default_srv;

        OutputResources out;
        out.output = output->default_uav;

        DXGPUDescriptorHandle gpu_handle = copy_global_resources_to_descriptor_heap(in, out);

        // Fill out description
        D3D12_DISPATCH_RAYS_DESC raytrace_desc;
        fill_out_ray_tracing_render_desc(binding_table.get_buffer(), raytrace_desc,
            output->width, output->height, 1,
            num_ray_types, binding_table.get_number_of_hit_groups());

        // Set up pipeline
        cl->set_descriptor_heap(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, descriptor_heap.descriptor_heap);

        cl->set_pipeline_state(pipeline.pipeline);
        cl->set_compute_root_signature(pipeline.root_signature);

        uint32 depth = min(settings.recursion_depth, settings.max_recursion_depth - 1);

        cl->set_compute_descriptor_table(PATH_TRACING_RS_RESOURCES, gpu_handle);
        cl->set_compute_dynamic_constant_buffer(PATH_TRACING_RS_CAMERA, common.camera_cbv);
        cl->set_compute_32bit_constants(PATH_TRACING_RS_CB,
            PathTracingCB{
                (uint32)dx_context.get_frame_id(),
                num_averaged_frames,
                depth,
                clamp(settings.start_russian_roulette_after, 0u, depth),
                (uint32)settings.use_thin_lens_camera,
                settings.focal_length,
                settings.focal_length / (2.f * settings.f_number),
                (uint32)settings.use_real_materials,
                (uint32)settings.enable_direct_lighting,
                settings.light_intensity_scale,
                settings.point_light_radius,
                (uint32)settings.multiple_importance_sampling,
            });

        cl->raytrace(raytrace_desc);

        ++num_averaged_frames;
    }

    void PathTracer::reset_rendering()
    {
        num_averaged_frames = 0;
    }
}