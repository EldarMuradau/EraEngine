// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core/threading.h"

#include "dx/dx.h"
#include "dx/dx_descriptor.h"

namespace era_engine
{
    struct DXDescriptorPage;

    struct DXDescriptorAllocation
    {
        inline NODISCARD CD3DX12_CPU_DESCRIPTOR_HANDLE cpu_at(uint32 index = 0) const
        {
            ASSERT(index < count);
            return CD3DX12_CPU_DESCRIPTOR_HANDLE(cpu_base, index, descriptor_size);
        }
        inline NODISCARD CD3DX12_GPU_DESCRIPTOR_HANDLE gpu_at(uint32 index = 0) const
        {
            ASSERT(index < count);
            return CD3DX12_GPU_DESCRIPTOR_HANDLE(gpu_base, index, descriptor_size);
        }

        NODISCARD inline bool valid() const { return count > 0; }

        uint64 count = 0;

    private:
        CD3DX12_CPU_DESCRIPTOR_HANDLE cpu_base;
        CD3DX12_GPU_DESCRIPTOR_HANDLE gpu_base;
        uint32 descriptor_size;
        uint32 page_index;

        friend struct DXDescriptorPage;
        friend struct DXDescriptorHeap;
    };

    struct DXDescriptorHeap
    {
        void initialize(D3D12_DESCRIPTOR_HEAP_TYPE type, bool shader_visible, uint64 page_size = 4096);

        DXDescriptorAllocation allocate(uint64 count = 1);
        void free(DXDescriptorAllocation allocation);

        com<ID3D12DescriptorHeap> get_heap(int index) const;

        D3D12_DESCRIPTOR_HEAP_TYPE type;

    private:
        std::mutex mutex;
        uint64 page_size;
        bool shader_visible;
        std::vector<DXDescriptorPage*> all_pages;
    };

    struct DXDescriptorRange
    {
        inline DXDoubleDescriptorHandle push_handle()
        {
            DXDoubleDescriptorHandle result =
                {
                    CD3DX12_CPU_DESCRIPTOR_HANDLE(base.cpu_handle, push_index, descriptor_handle_increment_size),
                    CD3DX12_GPU_DESCRIPTOR_HANDLE(base.gpu_handle, push_index, descriptor_handle_increment_size),
                };
            ++push_index;
            return result;
        }

        com<ID3D12DescriptorHeap> descriptor_heap;
        D3D12_DESCRIPTOR_HEAP_TYPE type;
        uint32 descriptor_handle_increment_size;

    private:
        DXDoubleDescriptorHandle base;

        uint32 max_num_descriptors;

        uint32 push_index;

        friend struct DXFrameDescriptorAllocator;
    };

    struct DXFrameDescriptorPage;

    struct DXFrameDescriptorAllocator
    {
        DXFrameDescriptorPage* used_pages[NUM_BUFFERED_FRAMES];
        DXFrameDescriptorPage* free_pages;

        std::mutex mutex;

        void initialize();
        void new_frame(uint32 buffered_frame_id);
        DXDescriptorRange allocate_contiguous_descriptor_range(uint32 count);
    };

    struct DXPushableDescriptorHeap
    {
        void initialize(uint32 max_size, bool shader_visible = true);
        void reset();
        DXCPUDescriptorHandle push();

        com<ID3D12DescriptorHeap> descriptor_heap;
        DXCPUDescriptorHandle current_cpu;
        DXGPUDescriptorHandle current_gpu;
    };
}