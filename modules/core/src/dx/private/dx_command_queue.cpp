// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "dx/dx_command_queue.h"
#include "dx/dx_command_list.h"
#include "dx/dx_context.h"

#include "core/threading.h"

namespace era_engine
{
    static DWORD process_running_command_lists(void* data);

    void DXCommandQueue::initialize(DXDevice device, D3D12_COMMAND_LIST_TYPE type)
    {
        fence_value = 0;
        command_list_type = type;

        D3D12_COMMAND_QUEUE_DESC desc = {};
        desc.Type = type;
        desc.Priority = D3D12_COMMAND_QUEUE_PRIORITY_NORMAL;
        desc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
        desc.NodeMask = 0;

        checkResult(device->CreateCommandQueue(&desc, IID_PPV_ARGS(&command_queue)));
        checkResult(device->CreateFence(fence_value, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)));

        time_stamp_frequency = 0; // Default value, if timing is not supported on this queue.
        time_stamp_to_cpu = 0;

        D3D12_FEATURE_DATA_D3D12_OPTIONS3 options3 = {};
        if (SUCCEEDED(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS3, &options3, sizeof(options3))))
        {
            if (type == D3D12_COMMAND_LIST_TYPE_COPY &&
                options3.CopyQueueTimestampQueriesSupported)
            {
                if (SUCCEEDED(command_queue->GetTimestampFrequency(&time_stamp_frequency)))
                {
                    uint64 gpu_timestamp, cpu_timestamp;
                    if (SUCCEEDED(command_queue->GetClockCalibration(&gpu_timestamp, &cpu_timestamp)))
                    {
                        if (gpu_timestamp > cpu_timestamp)
                        {
                            time_stamp_to_cpu = (int64)(gpu_timestamp - cpu_timestamp);
                        }
                        else
                        {
                            time_stamp_to_cpu = -(int64)(cpu_timestamp - gpu_timestamp);
                        }
                    }
                }
            }
        }

        process_thread_handle = CreateThread(0, 0, process_running_command_lists, this, 0, 0);

        switch (type)
        {
            case D3D12_COMMAND_LIST_TYPE_DIRECT:
                SET_NAME(command_queue, "Render command queue");
                break;
            case D3D12_COMMAND_LIST_TYPE_COMPUTE:
                SET_NAME(command_queue, "Compute command queue");
                break;
            case D3D12_COMMAND_LIST_TYPE_COPY:
                SET_NAME(command_queue, "Copy command queue");
                break;
        }
    }

    uint64 DXCommandQueue::signal()
    {
        std::lock_guard lock{submission_mutex};

        uint64 fence_value_for_signal = atomic_increment(fence_value) + 1;
        checkResult(command_queue->Signal(fence.Get(), fence_value_for_signal));

        return fence_value_for_signal;
    }

    NODISCARD bool DXCommandQueue::is_fence_complete(uint64 fence_value) const
    {
        return fence->GetCompletedValue() >= fence_value;
    }

    void DXCommandQueue::wait_for_fence(uint64 fence_value)
    {
        if (!is_fence_complete(fence_value))
        {
            HANDLE fence_event = CreateEvent(NULL, FALSE, FALSE, NULL);
            ASSERT(fence_event);

            fence->SetEventOnCompletion(fence_value, fence_event);
            WaitForSingleObject(fence_event, DWORD_MAX);

            CloseHandle(fence_event);
        }
    }

    void DXCommandQueue::wait_for_other_queue(DXCommandQueue& other)
    {
        command_queue->Wait(other.fence.Get(), other.signal());
    }

    void DXCommandQueue::wait_for_other_queue(DXCommandQueue& other, uint64 fence_value)
    {
        command_queue->Wait(other.fence.Get(), fence_value);
    }

    void DXCommandQueue::flush()
    {
        while (num_running_command_lists)
        {
        }

        wait_for_fence(signal());
    }

    static DWORD process_running_command_lists(void* data)
    {
        DXCommandQueue& queue = *(DXCommandQueue*)data;

        while (dx_context.running)
        {
            while (true)
            {
                queue.command_list_mutex.lock();
                DXCommandList* list = queue.running_command_lists;
                if (list)
                {
                    queue.running_command_lists = list->next;
                    if (list == queue.newest_running_command_list)
                    {
                        ASSERT(list->next == 0);
                        queue.newest_running_command_list = 0;
                    }
                }
                queue.command_list_mutex.unlock();

                if (list)
                {
                    queue.wait_for_fence(list->last_execution_fence_value);
                    list->reset();

                    queue.command_list_mutex.lock();

                    list->next = queue.free_command_lists;
                    queue.free_command_lists = list;

                    atomic_decrement(queue.num_running_command_lists);

                    queue.command_list_mutex.unlock();
                }
                else
                {
                    break;
                }
            }

            SwitchToThread(); // Yield.
        }

        return 0;
    }
}