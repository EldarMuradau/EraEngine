// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "dx/dx_descriptor_allocation.h"
#include "dx/dx_command_list.h"
#include "dx/dx_context.h"
#include "dx/dx_texture.h"

#include "core/block_allocator.h"

namespace era_engine
{
    struct DXDescriptorPage
    {
        DXDescriptorPage(D3D12_DESCRIPTOR_HEAP_TYPE type, uint64 capacity, bool shader_visible);

        std::pair<DXDescriptorAllocation, bool> allocate(uint64 count);
        void free(DXDescriptorAllocation allocation);

        D3D12_DESCRIPTOR_HEAP_TYPE type;
        com<ID3D12DescriptorHeap> descriptor_heap;

        CD3DX12_CPU_DESCRIPTOR_HANDLE cpu_base;
        CD3DX12_GPU_DESCRIPTOR_HANDLE gpu_base;

        uint32 descriptor_size;

        BlockAllocator allocator;
    };

    DXDescriptorPage::DXDescriptorPage(D3D12_DESCRIPTOR_HEAP_TYPE type, uint64 capacity, bool shader_visible)
    {
        allocator.initialize(capacity);

        D3D12_DESCRIPTOR_HEAP_DESC desc = {};
        desc.NumDescriptors = (uint32)capacity;
        desc.Type = type;
        if (shader_visible)
        {
            desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        }

        checkResult(dx_context.device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&descriptor_heap)));

        cpu_base = descriptor_heap->GetCPUDescriptorHandleForHeapStart();
        gpu_base = shader_visible ? descriptor_heap->GetGPUDescriptorHandleForHeapStart() : CD3DX12_GPU_DESCRIPTOR_HANDLE{};
        descriptor_size = dx_context.device->GetDescriptorHandleIncrementSize(type);
    }

    std::pair<DXDescriptorAllocation, bool> DXDescriptorPage::allocate(uint64 count)
    {
        uint64 offset = allocator.allocate(count);
        if (offset == UINT64_MAX)
        {
            return {DXDescriptorAllocation{}, false};
        }

        DXDescriptorAllocation result;
        result.count = count;
        result.cpu_base.InitOffsetted(cpu_base, (uint32)offset, descriptor_size);
        result.gpu_base.InitOffsetted(gpu_base, (uint32)offset, descriptor_size);
        result.descriptor_size = descriptor_size;

        return {result, true};
    }

    void DXDescriptorPage::free(DXDescriptorAllocation allocation)
    {
        uint64 offset = (allocation.cpu_base.ptr - cpu_base.ptr) / descriptor_size;
        allocator.free(offset, allocation.count);
    }

    void DXDescriptorHeap::initialize(D3D12_DESCRIPTOR_HEAP_TYPE type, bool shader_visible, uint64 page_size)
    {
        this->type = type;
        this->page_size = page_size;
        this->shader_visible = shader_visible;
    }

    DXDescriptorAllocation DXDescriptorHeap::allocate(uint64 count)
    {
        if (count == 0)
            return {};

        ASSERT(count <= page_size);

        // Lock section
        mutex.lock();

        for (uint32 i = 0; i < (uint32)all_pages.size(); ++i)
        {
            auto [allocation, success] = all_pages[i]->allocate(count);
            if (success)
            {
                allocation.page_index = i;
                mutex.unlock();
                return allocation;
            }
        }

        uint32 page_index = (uint32)all_pages.size();
        all_pages.push_back(new DXDescriptorPage(type, page_size, shader_visible));

        auto [allocation, success] = all_pages.back()->allocate(count);
        ASSERT(success);
        allocation.page_index = page_index;

        mutex.unlock();

        return allocation;
    }

    void DXDescriptorHeap::free(DXDescriptorAllocation allocation)
    {
        if (allocation.valid())
        {
            Lock lock{mutex};
            all_pages[allocation.page_index]->free(allocation);
        }
    }

    com<ID3D12DescriptorHeap> DXDescriptorHeap::get_heap(int index) const
    {
        return all_pages[index]->descriptor_heap;
    }

    struct DXFrameDescriptorPage
    {
        com<ID3D12DescriptorHeap> descriptor_heap;
        DXDoubleDescriptorHandle base;
        uint32 used_descriptors;
        uint32 max_num_descriptors;
        uint32 descriptor_handle_increment_size;

        DXFrameDescriptorPage* next;
    };

    void DXFrameDescriptorAllocator::initialize()
    {
    }

    void DXFrameDescriptorAllocator::new_frame(uint32 buffered_frame_id)
    {
        Lock lock{mutex};

        while (used_pages[buffered_frame_id])
        {
            DXFrameDescriptorPage* page = used_pages[buffered_frame_id];
            used_pages[buffered_frame_id] = page->next;
            page->next = free_pages;
            free_pages = page;
        }
    }

    DXDescriptorRange DXFrameDescriptorAllocator::allocate_contiguous_descriptor_range(uint32 count)
    {
        const uint32 current_frame = dx_context.get_buffered_frame_id();

        // Lock section
        mutex.lock();

        DXFrameDescriptorPage* current = used_pages[current_frame];
        if (!current || (current->max_num_descriptors - current->used_descriptors < count))
        {
            DXFrameDescriptorPage* free_page = free_pages;
            if (!free_page)
            {
                free_page = (DXFrameDescriptorPage*)calloc(1, sizeof(DXFrameDescriptorPage));

                D3D12_DESCRIPTOR_HEAP_DESC desc = {};
                desc.NumDescriptors = 1024;
                desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
                desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;

                checkResult(dx_context.device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&free_page->descriptor_heap)));

                free_page->max_num_descriptors = desc.NumDescriptors;
                free_page->descriptor_handle_increment_size = dx_context.device->GetDescriptorHandleIncrementSize(desc.Type);
                free_page->base.cpu_handle = free_page->descriptor_heap->GetCPUDescriptorHandleForHeapStart();
                free_page->base.gpu_handle = free_page->descriptor_heap->GetGPUDescriptorHandleForHeapStart();
            }

            free_pages = free_page->next;
            free_page->used_descriptors = 0;
            free_page->next = current;
            used_pages[current_frame] = free_page;
            current = free_page;
        }

        uint32 index = current->used_descriptors;
        current->used_descriptors += count;

        mutex.unlock();

        DXDescriptorRange result;

        result.base.cpu_handle = CD3DX12_CPU_DESCRIPTOR_HANDLE(current->base.cpu_handle, index, current->descriptor_handle_increment_size);
        result.base.gpu_handle = CD3DX12_GPU_DESCRIPTOR_HANDLE(current->base.gpu_handle, index, current->descriptor_handle_increment_size);
        result.descriptor_handle_increment_size = current->descriptor_handle_increment_size;
        result.descriptor_heap = current->descriptor_heap;
        result.max_num_descriptors = count;
        result.push_index = 0;

        return result;
    }

    void DXPushableDescriptorHeap::initialize(uint32 max_size, bool shader_visible)
    {
        D3D12_DESCRIPTOR_HEAP_DESC desc = {};
        desc.NumDescriptors = max_size;
        desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;

        if (shader_visible)
        {
            desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        }

        checkResult(dx_context.device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&descriptor_heap)));

        reset();
    }

    void DXPushableDescriptorHeap::reset()
    {
        current_cpu = descriptor_heap->GetCPUDescriptorHandleForHeapStart();
        current_gpu = descriptor_heap->GetGPUDescriptorHandleForHeapStart();
    }

    DXCPUDescriptorHandle DXPushableDescriptorHeap::push()
    {
        ++current_gpu;
        return current_cpu++;
    }

}