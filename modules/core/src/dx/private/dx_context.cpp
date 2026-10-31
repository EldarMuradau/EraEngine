// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "dx/dx_context.h"
#include "dx/dx_command_list.h"
#include "dx/dx_texture.h"
#include "dx/dx_buffer.h"
#include "dx/dx_profiling.h"

#include "core/string.h"
#include "core/frame_params.h"
#include "core/jobs/jobs.h"

#include "dlss/dlss.h"

#include <dx/D3D12MemAlloc.cpp>

extern "C"
{
    __declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}

namespace era_engine
{
    // Allocate dynamically on the heap to ensure that this thing never gets destroyed before anything else. This is quite a hack.
    DXContext& dx_context = *new DXContext{};

    LONG NTAPI handle_vectored_exception(PEXCEPTION_POINTERS exception_pointers)
    {
        PEXCEPTION_RECORD exception_record = exception_pointers->ExceptionRecord;

        switch (exception_record->ExceptionCode)
        {
            case DBG_PRINTEXCEPTION_WIDE_C:
            case DBG_PRINTEXCEPTION_C:

                if (exception_record->NumberParameters >= 2)
                {
                    ULONG len = (ULONG)exception_record->ExceptionInformation[0];

                    union
                    {
                        ULONG_PTR up;
                        PCWSTR pwz;
                        PCSTR psz;
                    };

                    up = exception_record->ExceptionInformation[1];

                    HANDLE h_out = GetStdHandle(STD_ERROR_HANDLE);

                    if (exception_record->ExceptionCode == DBG_PRINTEXCEPTION_C)
                    {
                        // Localized text will be incorrect displayed, if used not CP_OEMCP encoding.
                        // WriteConsoleA(hOut, psz, len, &len, 0);

                        // assume CP_ACP encoding
                        if (ULONG n = MultiByteToWideChar(CP_ACP, 0, psz, len, 0, 0))
                        {
                            PWSTR wz = (PWSTR)alloca(n * sizeof(WCHAR));

                            if (len = MultiByteToWideChar(CP_ACP, 0, psz, len, wz, n))
                            {
                                pwz = wz;
                            }
                        }
                    }

                    if (len)
                    {
                        WriteConsoleW(h_out, pwz, len - 1, &len, 0);
                    }
                }
                return EXCEPTION_CONTINUE_EXECUTION;
        }

        return EXCEPTION_CONTINUE_SEARCH;
    }

    static void enable_debug_layer()
    {
#if defined(_DEBUG)
        com<ID3D12Debug3> debugInterface;
        checkResult(D3D12GetDebugInterface(IID_PPV_ARGS(&debugInterface)));
        debugInterface->EnableDebugLayer();
        // debugInterface->SetEnableAutoDebugName(true);
        // debugInterface->SetEnableGPUBasedValidation(true);
#endif
    }

    static DXFactory create_factory()
    {
        DXFactory dxgi_factory;
        uint32 create_factory_flags = 0;
#if defined(_DEBUG)
        create_factory_flags = DXGI_CREATE_FACTORY_DEBUG;
#endif

        checkResult(CreateDXGIFactory2(create_factory_flags, IID_PPV_ARGS(&dxgi_factory)));

        return dxgi_factory;
    }

    struct AdapterDesc
    {
        DXAdapter adapter;
        D3D_FEATURE_LEVEL feature_level;
    };

    static AdapterDesc get_adapter(DXFactory factory, D3D_FEATURE_LEVEL minimum_feature_level = D3D_FEATURE_LEVEL_11_0)
    {
        com<IDXGIAdapter1> dxgi_adapter1;
        DXAdapter dxgi_adapter;
        DXGI_ADAPTER_DESC1 desc;

        D3D_FEATURE_LEVEL feature_level = D3D_FEATURE_LEVEL_9_1;

        D3D_FEATURE_LEVEL possible_feature_levels[] =
            {
                D3D_FEATURE_LEVEL_9_1,
                D3D_FEATURE_LEVEL_9_2,
                D3D_FEATURE_LEVEL_9_3,
                D3D_FEATURE_LEVEL_10_0,
                D3D_FEATURE_LEVEL_10_1,
                D3D_FEATURE_LEVEL_11_0,
                D3D_FEATURE_LEVEL_11_1,
                D3D_FEATURE_LEVEL_12_0,
                D3D_FEATURE_LEVEL_12_1};

        uint32 first_feature_level = 0;
        for (uint32 i = 0; i < arraysize(possible_feature_levels); ++i)
        {
            if (possible_feature_levels[i] == minimum_feature_level)
            {
                first_feature_level = i;
                break;
            }
        }

        uint64 max_dedicated_video_memory = 0;
        for (uint32 i = 0; factory->EnumAdapters1(i, &dxgi_adapter1) != DXGI_ERROR_NOT_FOUND; ++i)
        {
            DXGI_ADAPTER_DESC1 dxgi_adapter_desc1;
            dxgi_adapter1->GetDesc1(&dxgi_adapter_desc1);

            // Check to see if the adapter can create a D3D12 device without actually
            // creating it. Out of all adapters which support the minimum feature level,
            // the adapter with the largest dedicated video memory is favored.
            if ((dxgi_adapter_desc1.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) == 0)
            {
                D3D_FEATURE_LEVEL adapter_feature_level = D3D_FEATURE_LEVEL_9_1;
                bool supports_feature_level = false;

                for (uint32 fl = first_feature_level; fl < arraysize(possible_feature_levels); ++fl)
                {
                    if (SUCCEEDED(D3D12CreateDevice(dxgi_adapter1.Get(),
                            possible_feature_levels[fl], __uuidof(ID3D12Device), 0)))
                    {
                        adapter_feature_level = possible_feature_levels[fl];
                        supports_feature_level = true;
                    }
                }

                if (supports_feature_level && dxgi_adapter_desc1.DedicatedVideoMemory > max_dedicated_video_memory)
                {
                    checkResult(dxgi_adapter1.As(&dxgi_adapter));
                    max_dedicated_video_memory = dxgi_adapter_desc1.DedicatedVideoMemory;
                    feature_level = adapter_feature_level;
                    desc = dxgi_adapter_desc1;
                }
            }
        }

        return {dxgi_adapter, feature_level};
    }

    static DXDevice create_device(DXAdapter adapter, D3D_FEATURE_LEVEL feature_level)
    {
        DXDevice device;
        checkResult(D3D12CreateDevice(adapter.Get(), feature_level, IID_PPV_ARGS(&device)));

#if defined(_DEBUG)
        com<ID3D12InfoQueue> infoQueue;
        if (SUCCEEDED(device.As(&infoQueue)))
        {
            infoQueue->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_CORRUPTION, TRUE);
            infoQueue->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_ERROR, TRUE);
            infoQueue->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_WARNING, TRUE);

            // Suppress whole categories of messages.
            // D3D12_MESSAGE_CATEGORY categories[] = {};

            // Suppress messages based on their severity level.
            D3D12_MESSAGE_SEVERITY severities[] =
                {
                    D3D12_MESSAGE_SEVERITY_INFO};

            // Suppress individual messages by their ID.
            D3D12_MESSAGE_ID ids[] = {
                D3D12_MESSAGE_ID_CLEARRENDERTARGETVIEW_MISMATCHINGCLEARVALUE, // I'm really not sure how to avoid this message.
                D3D12_MESSAGE_ID_MAP_INVALID_NULLRANGE,                       // This warning occurs when using capture frame while graphics debugging.
                D3D12_MESSAGE_ID_UNMAP_INVALID_NULLRANGE,                     // This warning occurs when using capture frame while graphics debugging.
                D3D12_MESSAGE_ID_OBJECT_ACCESSED_WHILE_STILL_IN_USE,          // Disabled to allow OBS to capture the window.
            };

            D3D12_INFO_QUEUE_FILTER filter = {};
            // filter.DenyList.NumCategories = arraysize(categories);
            // filter.DenyList.pCategoryList = categories;
            filter.DenyList.NumSeverities = arraysize(severities);
            filter.DenyList.pSeverityList = severities;
            filter.DenyList.NumIDs = arraysize(ids);
            filter.DenyList.pIDList = ids;

            checkResult(infoQueue->PushStorageFilter(&filter));
        }
#endif

        return device;
    }

    static DXFeatureSupport check_feature_support(DXDevice device)
    {
        DXFeatureSupport result = {};

        D3D12_FEATURE_DATA_D3D12_OPTIONS5 options5 = {};
        if (SUCCEEDED(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS5, &options5, sizeof(options5))))
        {
            result.raytracing_tier = (DXRaytracingTier)options5.RaytracingTier;
        }

#ifdef SDK_SUPPORTS_MESH_SHADERS
        D3D12_FEATURE_DATA_D3D12_OPTIONS7 options7 = {};
        if (SUCCEEDED(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS7, &options7, sizeof(options7))))
        {
            result.mesh_shader_tier = (DXMeshShaderTier)options7.MeshShaderTier;
        }
        else
        {
            std::cerr << "Checking support for mesh shader feature failed. Maybe you need to update your Windows version.\n";
        }
#endif

        D3D12_FEATURE_DATA_D3D12_OPTIONS options1 = {};
        if (SUCCEEDED(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS, &options1, sizeof(options1))))
        {
            result.tiling_tier = (DXTilingTier)options1.TiledResourcesTier;
        }

        return result;
    }

    bool DXContext::initialize()
    {
        SetThreadDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
        AddVectoredExceptionHandler(TRUE, handle_vectored_exception);

        checkResult(CoInitializeEx(NULL, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE));

        enable_debug_layer();

        factory = create_factory();
        AdapterDesc adapter_desc = get_adapter(factory);
        this->adapter = adapter_desc.adapter;

        if (!adapter)
        {
            std::cerr << "No DX12 capable GPU found.\n";
            return false;
        }

        device = create_device(adapter, adapter_desc.feature_level);

#if USE_D3D12_BLOCK_ALLOCATOR
        D3D12MA::ALLOCATOR_DESC allocator_desc = {};
        allocator_desc.pDevice = device.Get();
        allocator_desc.pAdapter = adapter.Get();
        checkResult(D3D12MA::CreateAllocator(&allocator_desc, &memory_allocator));
#endif

        feature_support = check_feature_support(device);

        if (!era_engine::dlss::check_dlss_status(adapter.Get()))
            feature_support.dlss_status = DLSS_STATUS_NOT_AVAILABLE;

        buffered_frame_id = NUM_BUFFERED_FRAMES - 1;

        descriptor_handle_increment_size = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

        srv_uav_allocator.initialize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, false, 4096);
        srv_uav_allocator_shader_visible.initialize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, true, 128);
        rtv_allocator.initialize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV, false, 1024);
        dsv_allocator.initialize(D3D12_DESCRIPTOR_HEAP_TYPE_DSV, false, 1024);
        frame_descriptor_allocator.initialize();

        for (uint32 i = 0; i < NUM_BUFFERED_FRAMES; ++i)
        {
            page_pools[i].initialize(mb(32));
            frame_upload_buffers[i].page_pool = &page_pools[i];

#if ENABLE_DX_PROFILING
            timestampHeaps[i].initialize(MAX_NUM_DX_PROFILE_EVENTS);
            resolvedTimestampBuffers[i] = create_readback_buffer(sizeof(uint64), MAX_NUM_DX_PROFILE_EVENTS);
#endif
        }

        render_queue.initialize(device, D3D12_COMMAND_LIST_TYPE_DIRECT);
        compute_queue.initialize(device, D3D12_COMMAND_LIST_TYPE_COMPUTE);
        copy_queue.initialize(device, D3D12_COMMAND_LIST_TYPE_COPY);

        D3D12_INDIRECT_ARGUMENT_DESC argument_desc;
        argument_desc.Type = D3D12_INDIRECT_ARGUMENT_TYPE_DISPATCH;
        default_dispatch_command_signature = create_command_signature({}, &argument_desc, 1, sizeof(D3D12_DISPATCH_ARGUMENTS));

        return true;
    }

    void DXContext::flush_application()
    {
        render_queue.flush();
        compute_queue.flush();
        copy_queue.flush();
    }

    void DXContext::quit()
    {
#if ENABLE_DX_PROFILING
        for (uint32 b = 0; b < NUM_BUFFERED_FRAMES; ++b)
        {
            resolvedTimestampBuffers[b].reset();
        }
#endif

        running = false;
        flush_application();
        WaitForSingleObject(render_queue.process_thread_handle, INFINITE);
        WaitForSingleObject(compute_queue.process_thread_handle, INFINITE);
        WaitForSingleObject(copy_queue.process_thread_handle, INFINITE);

        for (uint32 b = 0; b < NUM_BUFFERED_FRAMES; ++b)
        {
            texture_graveyard[b].clear();
            buffer_graveyard[b].clear();
            object_graveyard[b].clear();

            for (auto allocation : allocation_graveyard[b])
            {
                allocation->Release();
            }
            allocation_graveyard[b].clear();
        }
    }

    void DXContext::retire(TextureGrave&& texture)
    {
        Lock lock{mutex};
        texture_graveyard[get_buffered_frame_id()].push_back(std::move(texture));
    }

    void DXContext::retire(BufferGrave&& buffer)
    {
        Lock lock{mutex};
        buffer_graveyard[get_buffered_frame_id()].push_back(std::move(buffer));
    }

    void DXContext::retire(DXObject obj)
    {
        Lock lock{mutex};
        object_graveyard[get_buffered_frame_id()].push_back(obj);
    }

    void DXContext::retire(D3D12MA::Allocation* allocation)
    {
        Lock lock{mutex};
        allocation_graveyard[get_buffered_frame_id()].push_back(allocation);
    }

    DXCommandQueue& DXContext::get_queue(D3D12_COMMAND_LIST_TYPE type)
    {
        return type == D3D12_COMMAND_LIST_TYPE_DIRECT ? render_queue : type == D3D12_COMMAND_LIST_TYPE_COMPUTE ? compute_queue
                                                                                                               : copy_queue;
    }

    DXCommandList* DXContext::get_free_command_list(DXCommandQueue& queue)
    {
        queue.command_list_mutex.lock();
        DXCommandList* result = queue.free_command_lists;
        if (result)
        {
            queue.free_command_lists = result->next;
        }
        queue.command_list_mutex.unlock();

        if (!result)
        {
            result = new DXCommandList(queue.command_list_type);
            atomic_increment(queue.total_num_command_lists);
        }

#if ENABLE_DX_PROFILING
        result->time_stamp_query_heap = timestampHeaps[get_buffered_frame_id()].heap;
#endif

        return result;
    }

    DXCommandList* DXContext::get_free_copy_command_list()
    {
        return get_free_command_list(copy_queue);
    }

    DXCommandList* DXContext::get_free_compute_command_list(bool async)
    {
        return get_free_command_list(async ? compute_queue : render_queue);
    }

    DXCommandList* DXContext::get_free_render_command_list()
    {
        DXCommandList* cl = get_free_command_list(render_queue);
        CD3DX12_RECT scissor_rect(0, 0, LONG_MAX, LONG_MAX);
        cl->set_scissor(scissor_rect);
        return cl;
    }

    static void enqueue_running_command_list(DXCommandQueue& queue, DXCommandList* command_list)
    {
        command_list->next = 0;
        if (queue.newest_running_command_list)
        {
            ASSERT(queue.newest_running_command_list->next == 0);
            queue.newest_running_command_list->next = command_list;
        }
        queue.newest_running_command_list = command_list;
        if (!queue.running_command_lists)
        {
            queue.running_command_lists = command_list;
        }
    }

    uint64 DXContext::execute_command_list(DXCommandList* command_list)
    {
        DXCommandQueue& queue = get_queue(command_list->type);

        checkResult(command_list->command_list->Close());

        ID3D12CommandList* d3d12_list = command_list->command_list.Get();

        uint64 fence_value;
        {
            std::lock_guard submission_lock{queue.submission_mutex};
            queue.command_queue->ExecuteCommandLists(1, &d3d12_list);
            fence_value = queue.signal();
        }

        command_list->last_execution_fence_value = fence_value;

        {
            Lock lock{queue.command_list_mutex};
            enqueue_running_command_list(queue, command_list);
            atomic_increment(queue.num_running_command_lists);
        }


        return fence_value;
    }

    uint64 DXContext::execute_command_lists(DXCommandList** command_lists, uint32 count)
    {
        DXCommandQueue& queue = get_queue(command_lists[0]->type);

        ID3D12CommandList* d3d12_lists[16];

        for (uint32 i = 0; i < count; ++i)
        {
            checkResult(command_lists[i]->command_list->Close());
            d3d12_lists[i] = command_lists[i]->command_list.Get();
        }

        uint64 fence_value;
        {
            std::lock_guard submission_lock{queue.submission_mutex};
            queue.command_queue->ExecuteCommandLists(count, d3d12_lists);
            fence_value = queue.signal();
        }

        for (uint32 i = 0; i < count; ++i)
        {
            command_lists[i]->last_execution_fence_value = fence_value;
        }

        {
            Lock lock{queue.command_list_mutex};
            for (uint32 i = 0; i < count; ++i)
            {
                enqueue_running_command_list(queue, command_lists[i]);
            }
            atomic_add(queue.num_running_command_lists, count);
        }

        return fence_value;
    }

    DXAllocation DXContext::allocate_dynamic_buffer(uint32 size_in_bytes, uint32 alignment)
    {
        const uint32 slot = get_buffered_frame_id();

        Lock lock{upload_mutexes[slot]};
        DXAllocation allocation = frame_upload_buffers[slot].allocate(size_in_bytes, alignment);
        return allocation;
    }

    std::pair<DXDynamicConstantBuffer, void*> DXContext::upload_dynamic_constant_buffer(uint32 size_in_bytes, const void* data)
    {
        DXAllocation allocation = allocate_dynamic_buffer(size_in_bytes, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT);
        memcpy(allocation.cpu_ptr, data, size_in_bytes);
        return {{allocation.gpu_ptr}, allocation.cpu_ptr};
    }

    std::pair<DXDynamicVertexBuffer, void*> DXContext::create_dynamic_vertex_buffer(uint32 element_size, uint32 element_count)
    {
        uint32 size_in_bytes = element_size * element_count;
        DXAllocation allocation = allocate_dynamic_buffer(size_in_bytes, element_size);

        D3D12_VERTEX_BUFFER_VIEW vertex_buffer_view = {};
        vertex_buffer_view.BufferLocation = allocation.gpu_ptr;
        vertex_buffer_view.SizeInBytes = size_in_bytes;
        vertex_buffer_view.StrideInBytes = element_size;

        return {{vertex_buffer_view}, allocation.cpu_ptr};
    }

    std::pair<DXDynamicIndexBuffer, void*> DXContext::create_dynamic_index_buffer(uint32 element_size, uint32 element_count)
    {
        uint32 size_in_bytes = element_size * element_count;
        DXAllocation allocation = allocate_dynamic_buffer(size_in_bytes, element_size);

        D3D12_INDEX_BUFFER_VIEW index_buffer_view = {};
        index_buffer_view.BufferLocation = allocation.gpu_ptr;
        index_buffer_view.SizeInBytes = size_in_bytes;
        index_buffer_view.Format = get_index_buffer_format(element_size);

        return {{index_buffer_view}, allocation.cpu_ptr};
    }

    DXMemoryUsage DXContext::get_memory_usage()
    {
        DXGI_QUERY_VIDEO_MEMORY_INFO memory_info;
        checkResult(adapter->QueryVideoMemoryInfo(0, DXGI_MEMORY_SEGMENT_GROUP_LOCAL, &memory_info));

        return {(uint32)byte_to_mb(memory_info.CurrentUsage), (uint32)byte_to_mb(memory_info.Budget)};
    }

    void DXContext::end_frame(DXCommandList* cl)
    {
#if ENABLE_DX_PROFILING
        dxProfilingFrameEndMarker(cl);
        const uint32 slot = get_buffered_frame_id();
        cl->command_list->ResolveQueryData(timestampHeaps[slot].heap.Get(), D3D12_QUERY_TYPE_TIMESTAMP, 0, timestampQueryIndex[slot], resolvedTimestampBuffers[slot]->resource.Get(), 0);
#endif
    }

    void DXContext::new_frame(uint64 frame_id)
    {
        this->frame_id = frame_id;

        Lock lock{mutex};
        buffered_frame_id = (uint32)(frame_id % NUM_BUFFERED_FRAMES);

#if ENABLE_DX_PROFILING
        uint64* timestamps = (uint64*)map_buffer(resolvedTimestampBuffers[buffered_frame_id], true);
        dxProfilingResolveTimeStamps(timestamps);
        unmap_buffer(resolvedTimestampBuffers[buffered_frame_id], false);

        timestampQueryIndex[buffered_frame_id] = 0;
#endif

        texture_graveyard[buffered_frame_id].clear();
        buffer_graveyard[buffered_frame_id].clear();
        object_graveyard[buffered_frame_id].clear();
        for (auto allocation : allocation_graveyard[buffered_frame_id])
        {
            allocation->Release();
        }
        allocation_graveyard[buffered_frame_id].clear();

        {
            Lock upload_lock{upload_mutexes[buffered_frame_id]};
            frame_upload_buffers[buffered_frame_id].reset();
            page_pools[buffered_frame_id].reset();
        }
        frame_descriptor_allocator.new_frame(buffered_frame_id);
    }

    uint64 DXContext::get_frame_id() const
    {
        const FrameParams* frame = JobSystem::get_frame_context();
        return frame != nullptr ? frame->frame_id : frame_id;
    }

    uint32 DXContext::get_buffered_frame_id() const
    {
        const FrameParams* frame = JobSystem::get_frame_context();
        return frame != nullptr ? frame->slot : buffered_frame_id;
    }

    ERA_CORE_API std::pair<DXDynamicVertexBuffer, void*> create_dynamic_vertex_buffer(uint32 element_size, uint32 element_count)
    {
        return dx_context.create_dynamic_vertex_buffer(element_size, element_count);
    }

    ERA_CORE_API std::pair<DXDynamicIndexBuffer, void*> create_dynamic_index_buffer(uint32 element_size, uint32 element_count)
    {
        return dx_context.create_dynamic_index_buffer(element_size, element_count);
    }

    DXMemoryUsage get_dx_mem_usage()
    {
        return dx_context.get_memory_usage();
    }

    ERA_CORE_API DXContext* get_dx_context()
    {
        return &dx_context;
    }
}