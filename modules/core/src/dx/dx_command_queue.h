// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "dx/dx.h"

namespace era_engine
{
    struct DXCommandList;

    struct DXCommandQueue
    {
        void initialize(DXDevice device, D3D12_COMMAND_LIST_TYPE type);

        uint64 signal();
        NODISCARD bool is_fence_complete(uint64 fence_value) const;
        void wait_for_fence(uint64 fence_value);
        void wait_for_other_queue(DXCommandQueue& other);
        void wait_for_other_queue(DXCommandQueue& other, uint64 fence_value);
        void flush();

        uint64 time_stamp_frequency; // In Hz.
        int64 time_stamp_to_cpu;     // GPU queue - CPU.

        D3D12_COMMAND_LIST_TYPE command_list_type;
        com<ID3D12CommandQueue> command_queue;
        com<ID3D12Fence> fence;
        volatile uint64 fence_value;

        DXCommandList* running_command_lists;
        DXCommandList* newest_running_command_list;
        DXCommandList* free_command_lists;

        volatile uint32 num_running_command_lists;
        volatile uint32 total_num_command_lists; // Used only for validation.

        HANDLE process_thread_handle;

        std::mutex command_list_mutex;

        // ExecuteCommandLists + Signal as one step, so fence values must reach the queue in increasing order.
        std::recursive_mutex submission_mutex;
    };
}