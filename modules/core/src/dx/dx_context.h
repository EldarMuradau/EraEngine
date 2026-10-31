#pragma once

#include "core_api.h"

#include "core/threading.h"

#include "dx/dx.h"
#include "dx/dx_command_queue.h"
#include "dx/dx_upload_buffer.h"
#include "dx/dx_descriptor_allocation.h"
#include "dx/dx_buffer.h"
#include "dx/dx_query.h"

namespace era_engine
{
    struct ERA_CORE_API DXMemoryUsage
    {
        // In MB.
        uint32 currently_used;
        uint32 available;
    };

    enum DXRaytracingTier
    {
        DX_RAYTRACING_NOT_AVAILABLE,
        DX_RAYTRACING_1_0,
        DX_RAYTRACING_1_1
    };

    enum DLSSStatus
    {
        DLSS_STATUS_NOT_AVAILABLE,
        DLSS_STATUS_FSR_WRAPPER,
        DLSS_STATUS_3_0,
        DLSS_STATUS_3_5
    };

    enum DXMeshShaderTier
    {
        DX_MESH_SHADER_NOT_AVAILABLE,
        DX_MESH_SHADER_1_0
    };

    enum DXTilingTier
    {
        DX_TILING_NOT_AVAILABLE,
        DX_TILING_1,
        DX_TILING_2,
        DX_TILING_3,
        DX_TILING_4
    };

    struct ERA_CORE_API DXFeatureSupport
    {
        DXRaytracingTier raytracing_tier = DX_RAYTRACING_1_0;
        DXMeshShaderTier mesh_shader_tier = DX_MESH_SHADER_1_0;
        DXTilingTier tiling_tier = DX_TILING_1;
        DLSSStatus dlss_status = DLSS_STATUS_FSR_WRAPPER;

        constexpr bool raytracing() const { return raytracing_tier >= DX_RAYTRACING_1_0; }
        constexpr bool mesh_shaders() const { return mesh_shader_tier >= DX_MESH_SHADER_1_0; }
        constexpr bool tiling() const { return tiling_tier >= DX_TILING_1; }
        constexpr bool dlss() const { return dlss_status >= DLSS_STATUS_FSR_WRAPPER; }
    };

    struct DXContext
    {
        bool initialize();
        void quit();

        void new_frame(uint64 frame_id);
        void end_frame(DXCommandList* cl);
        void flush_application();

        DXCommandList* get_free_copy_command_list();
        DXCommandList* get_free_compute_command_list(bool async);
        DXCommandList* get_free_render_command_list();
        uint64 execute_command_list(DXCommandList* command_list);
        uint64 execute_command_lists(DXCommandList** command_lists, uint32 count);

        // Careful with these functions. They are not thread safe.
        DXAllocation allocate_dynamic_buffer(uint32 size_in_bytes, uint32 alignment = D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT);

        std::pair<DXDynamicConstantBuffer, void*> upload_dynamic_constant_buffer(uint32 size_in_bytes, const void* data);
        template <typename T>
        DXDynamicConstantBuffer upload_dynamic_constant_buffer(const T& data)
        {
            return upload_dynamic_constant_buffer(sizeof(T), &data).first;
        }

        std::pair<DXDynamicVertexBuffer, void*> create_dynamic_vertex_buffer(uint32 element_size, uint32 element_count);
        std::pair<DXDynamicIndexBuffer, void*> create_dynamic_index_buffer(uint32 element_size, uint32 element_count);

        DXMemoryUsage get_memory_usage();

        DXFactory factory;
        DXAdapter adapter;
        DXDevice device;

        DXCommandQueue render_queue;
        DXCommandQueue compute_queue;
        DXCommandQueue copy_queue;

        DXFeatureSupport feature_support;

        // Frame of the calling code, otherwise the last started frame.
        uint64 get_frame_id() const;
        uint32 get_buffered_frame_id() const;

        DXDescriptorHeap srv_uav_allocator;
        DXDescriptorHeap srv_uav_allocator_shader_visible; // Used for resources, which can be UAV cleared.

        DXDescriptorHeap rtv_allocator;
        DXDescriptorHeap dsv_allocator;

#if ENABLE_DX_PROFILING
        volatile uint32 timestampQueryIndex[NUM_BUFFERED_FRAMES];
#endif

        DXUploadBuffer frame_upload_buffers[NUM_BUFFERED_FRAMES];

        DXFrameDescriptorAllocator frame_descriptor_allocator;

        volatile bool running = true;

        uint32 descriptor_handle_increment_size;

        D3D12MA::Allocator* memory_allocator;

        void retire(struct TextureGrave&& texture);
        void retire(struct BufferGrave&& buffer);
        void retire(DXObject obj);
        void retire(D3D12MA::Allocation* allocation);

        DXCommandSignature default_dispatch_command_signature;

    private:
#if ENABLE_DX_PROFILING
        DXTimestampQueryHeap timestampHeaps[NUM_BUFFERED_FRAMES];
        ref<DXBuffer> resolvedTimestampBuffers[NUM_BUFFERED_FRAMES];
#endif

        DXPagePool page_pools[NUM_BUFFERED_FRAMES];
        std::mutex upload_mutexes[NUM_BUFFERED_FRAMES];

        std::mutex mutex;

        // Last started frame.
        uint64 frame_id = 0;
        uint32 buffered_frame_id = 0;

        std::vector<struct TextureGrave> texture_graveyard[NUM_BUFFERED_FRAMES];
        std::vector<struct BufferGrave> buffer_graveyard[NUM_BUFFERED_FRAMES];
        std::vector<DXObject> object_graveyard[NUM_BUFFERED_FRAMES];
        std::vector<D3D12MA::Allocation*> allocation_graveyard[NUM_BUFFERED_FRAMES];

        DXCommandQueue& get_queue(D3D12_COMMAND_LIST_TYPE type);
        DXCommandList* get_free_command_list(DXCommandQueue& queue);

        friend struct DLSSFeatureAdapter;
    };

    extern DXContext& dx_context;

    ERA_CORE_API DXContext* get_dx_context();

    ERA_CORE_API std::pair<DXDynamicVertexBuffer, void*> create_dynamic_vertex_buffer(uint32 element_size, uint32 element_count);
    ERA_CORE_API std::pair<DXDynamicIndexBuffer, void*> create_dynamic_index_buffer(uint32 element_size, uint32 element_count);

    ERA_CORE_API DXMemoryUsage get_dx_mem_usage();
}