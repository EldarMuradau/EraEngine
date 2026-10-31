// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "dx/dx_descriptor_allocation.h"
#include "dx/dx_command_list.h"

#include "rendering/raytracing.h"
#include "rendering/raytracing_tlas.h"
#include "rendering/raytracing_binding_table.h"
#include "rendering/material.h"

namespace era_engine
{
    struct DXRaytracer
    {
    protected:
        void fill_out_ray_tracing_render_desc(const ref<DXBuffer>& binding_table_buffer,
            D3D12_DISPATCH_RAYS_DESC& raytrace_desc,
            uint32 render_width, uint32 render_height, uint32 render_depth,
            uint32 num_ray_types, uint32 num_hit_groups);

        template <typename InputResources, typename OutputResources>
        DXGPUDescriptorHandle copy_global_resources_to_descriptor_heap(const InputResources& in, const OutputResources& out);

        template <typename InputResources, typename OutputResources>
        void allocate_descriptor_heap_space_for_global_resources(DXPushableDescriptorHeap& descriptor_heap);

        DXRaytracingPipeline pipeline;

        DXCPUDescriptorHandle resource_cpu_base[NUM_BUFFERED_FRAMES];
        DXGPUDescriptorHandle resource_gpu_base[NUM_BUFFERED_FRAMES];
    };

    template <typename InputResources, typename OutputResources>
    inline DXGPUDescriptorHandle DXRaytracer::copy_global_resources_to_descriptor_heap(const InputResources& in, const OutputResources& out)
    {
        DXCPUDescriptorHandle cpu_handle = resource_cpu_base[dx_context.get_buffered_frame_id()];
        DXGPUDescriptorHandle gpu_handle = resource_gpu_base[dx_context.get_buffered_frame_id()];

        const uint32 num_input_resources = sizeof(InputResources) / sizeof(DXCPUDescriptorHandle);
        const uint32 num_output_resources = sizeof(OutputResources) / sizeof(DXCPUDescriptorHandle);
        const uint32 total_num_resources = num_input_resources + num_output_resources;

        D3D12_CPU_DESCRIPTOR_HANDLE handles[total_num_resources];
        memcpy(handles, &in, sizeof(InputResources));
        memcpy((D3D12_CPU_DESCRIPTOR_HANDLE*)handles + num_input_resources, &out, sizeof(OutputResources));

        dx_context.device->CopyDescriptors(
            1, (D3D12_CPU_DESCRIPTOR_HANDLE*)&cpu_handle, &total_num_resources,
            total_num_resources, handles, nullptr,
            D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

        return gpu_handle;
    }

    template <typename InputResources, typename OutputResources>
    inline void DXRaytracer::allocate_descriptor_heap_space_for_global_resources(DXPushableDescriptorHeap& descriptor_heap)
    {
        const uint32 num_input_resources = sizeof(InputResources) / sizeof(DXCPUDescriptorHandle);
        const uint32 num_output_resources = sizeof(OutputResources) / sizeof(DXCPUDescriptorHandle);
        const uint32 total_num_resources = num_input_resources + num_output_resources;

        for (uint32 i = 0; i < NUM_BUFFERED_FRAMES; ++i)
        {
            resource_cpu_base[i] = descriptor_heap.current_cpu;
            resource_gpu_base[i] = descriptor_heap.current_gpu;

            for (uint32 j = 0; j < total_num_resources; ++j)
                descriptor_heap.push();
        }
    }
}