// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#define PROFILING_INTERNAL

#include "core/cpu_profiling.h"
#include "core/imgui.h"

// #include "dx/dx_context.h"

namespace era_engine
{
    bool cpu_profiler_window_open = false;
}

#if ENABLE_CPU_PROFILING

#define MAX_NUM_CPU_PROFILE_THREADS 128
#define MAX_NUM_CPU_PROFILE_FRAMES 1024

namespace era_engine
{
    std::atomic<uint32> cpu_profile_index;
    std::atomic<uint32> cpu_profile_completely_written[2];
    ProfileEvent cpu_profile_events[2][MAX_NUM_CPU_PROFILE_EVENTS];
    ProfileStat cpu_profile_stats[2][MAX_NUM_CPU_PROFILE_STATS];

    struct CpuProfileFrame : ProfileFrame
    {
        uint16 first_top_level_block_per_thread[MAX_NUM_CPU_PROFILE_THREADS];

        ProfileBlock profile_block_pool[MAX_NUM_CPU_PROFILE_BLOCKS];
        uint32 total_num_profile_blocks;

        ProfileStat stats[MAX_NUM_CPU_PROFILE_STATS];
        uint32 num_stats;
    };

    static uint32 profile_threads[MAX_NUM_CPU_PROFILE_THREADS];
    static char profile_thread_names[MAX_NUM_CPU_PROFILE_THREADS][64];
    static uint32 num_threads;

    static CpuProfileFrame profile_frames[MAX_NUM_CPU_PROFILE_FRAMES];
    static uint32 profile_frame_write_index;

    static CpuProfileFrame dummy_frames[2];
    static uint32 dummy_frame_write_index;

    static bool pause_recording;

    static uint16 stack[MAX_NUM_CPU_PROFILE_THREADS][1024];
    static uint32 depth[MAX_NUM_CPU_PROFILE_THREADS];

    static uint32 map_thread_id_to_index(uint32 thread_id)
    {
        for (uint32 i = 0; i < num_threads; ++i)
        {
            if (profile_threads[i] == thread_id)
            {
                return i;
            }
        }

        HANDLE handle = OpenThread(THREAD_ALL_ACCESS, false, thread_id);
        ASSERT(handle);
        WCHAR* description = nullptr;
        checkResult(GetThreadDescription(handle, &description));
        CloseHandle(handle);

        if (!description || !description[0])
        {
            description = (WCHAR*)L"Main thread";
        }

        ASSERT(num_threads < MAX_NUM_CPU_PROFILE_THREADS);
        uint32 index = num_threads++;
        profile_threads[index] = thread_id;
        snprintf(profile_thread_names[index], sizeof(profile_thread_names[index]), "Thread %u (%ws)", thread_id, description);
        return index;
    }

    static void initialize_new_frame(CpuProfileFrame& old_frame, CpuProfileFrame& new_frame)
    {
        for (uint32 thread = 0; thread < MAX_NUM_CPU_PROFILE_THREADS; ++thread)
        {
            if (depth[thread] > 0)
            {
                // Some blocks are still running on this thread.
                copy_profile_blocks(old_frame.profile_block_pool, stack[thread], depth[thread], new_frame.profile_block_pool, new_frame.total_num_profile_blocks);
                new_frame.first_top_level_block_per_thread[thread] = stack[thread][0];
            }
            else
            {
                new_frame.first_top_level_block_per_thread[thread] = INVALID_PROFILE_BLOCK;
                stack[thread][0] = INVALID_PROFILE_BLOCK;
            }
        }
    }

    void cpu_profiling_resolve_time_stamps()
    {
        uint32 current_frame = profile_frame_write_index;

        uint32 array_index = _CPU_PROFILE_GET_ARRAY_INDEX(cpu_profile_index);       // We are only interested in the most significant bit here, so don't worry about thread safety.
        uint32 current_index = cpu_profile_index.exchange((1 - array_index) << 31); // Swap array and get current event count.

        ProfileEvent* events = cpu_profile_events[array_index];
        uint32 num_events = _CPU_PROFILE_GET_EVENT_INDEX(current_index);
        auto stats = cpu_profile_stats[array_index];
        uint32 num_stats = _CPU_PROFILE_GET_STAT_INDEX(current_index);
        uint32 num_writes = num_events + num_stats;

        while (num_writes > cpu_profile_completely_written[array_index])
        {
        } // Wait until all events and stats have been written completely.
        cpu_profile_completely_written[array_index] = 0;

        static bool initialized_stack = false;

        // Initialize on the very first frame.
        if (!initialized_stack)
        {
            for (uint32 thread = 0; thread < MAX_NUM_CPU_PROFILE_THREADS; ++thread)
            {
                stack[thread][0] = INVALID_PROFILE_BLOCK;
                profile_frames[0].first_top_level_block_per_thread[thread] = INVALID_PROFILE_BLOCK;
                depth[thread] = 0;
            }

            initialized_stack = true;
        }

        CPU_PROFILE_BLOCK("CPU Profiling"); // Important: Must be after array swap!

        {
            CPU_PROFILE_BLOCK("Collate profile events from last frame");

            std::stable_sort(events, events + num_events, [](const ProfileEvent& a, const ProfileEvent& b)
                {
                    return a.timestamp < b.timestamp;
                });


            CpuProfileFrame* frame = !pause_recording ? (profile_frames + profile_frame_write_index) : (dummy_frames + dummy_frame_write_index);

            for (uint32 i = 0; i < num_events; ++i)
            {
                ProfileEvent* event = events + i;
                uint32 thread_id = event->thread_id;
                uint32 thread_index = map_thread_id_to_index(thread_id);

                uint32 blocks_before = frame->total_num_profile_blocks;

                uint64 frame_end_timestamp = 0;
                if (handle_profile_event(events, i, num_events, stack[thread_index], depth[thread_index], frame->profile_block_pool, frame->total_num_profile_blocks, frame_end_timestamp, false))
                {
                    static uint64 clock_frequency;
                    static bool performance_frequency_queried = QueryPerformanceFrequency((LARGE_INTEGER*)&clock_frequency);

                    CpuProfileFrame* previous_frame;
                    if (!pause_recording)
                    {
                        uint32 previous_frame_index = (profile_frame_write_index == 0) ? (MAX_NUM_CPU_PROFILE_FRAMES - 1) : (profile_frame_write_index - 1);
                        previous_frame = profile_frames + previous_frame_index;
                    }
                    else
                    {
                        uint32 previous_frame_index = 1 - dummy_frame_write_index;
                        previous_frame = dummy_frames + previous_frame_index;
                    }

                    frame->start_clock = (previous_frame->end_clock == 0) ? frame_end_timestamp : previous_frame->end_clock;
                    frame->end_clock = frame_end_timestamp;

                    frame->global_frame_id = dx_context.get_frame_id();

                    frame->duration = (float)(frame->end_clock - frame->start_clock) / clock_frequency * 1000.f;

                    for (uint32 i = 0; i < frame->total_num_profile_blocks; ++i)
                    {
                        ProfileBlock* block = frame->profile_block_pool + i;

                        uint64 end_clock = (block->end_clock == 0) ? frame->end_clock : block->end_clock;
                        ASSERT(end_clock <= frame->end_clock);

                        if (block->start_clock >= frame->start_clock)
                        {
                            block->rel_start = (float)(block->start_clock - frame->start_clock) / clock_frequency * 1000.f;
                        }
                        else
                        {
                            // For blocks which started in a previous frame.
                            block->rel_start = -(float)(frame->start_clock - block->start_clock) / clock_frequency * 1000.f;
                        }
                        block->duration = (float)(end_clock - block->start_clock) / clock_frequency * 1000.f;
                    }

                    frame->num_stats = num_stats;
                    memcpy(frame->stats, stats, num_stats * sizeof(ProfileStat));

                    CpuProfileFrame* old_frame = frame;

                    if (!pause_recording)
                    {
                        profile_frame_write_index = (profile_frame_write_index + 1) % MAX_NUM_CPU_PROFILE_FRAMES;
                        frame = profile_frames + profile_frame_write_index;
                    }
                    else
                    {
                        dummy_frame_write_index = 1 - dummy_frame_write_index;
                        frame = dummy_frames + dummy_frame_write_index;
                    }

                    frame->total_num_profile_blocks = 0;

                    initialize_new_frame(*old_frame, *frame);
                }
                else
                {
                    // Set first top-level block if another block was added and this thread has no first top-level block yet.
                    uint32 blocks_after = frame->total_num_profile_blocks;

                    if (blocks_before != blocks_after && frame->first_top_level_block_per_thread[thread_index] == INVALID_PROFILE_BLOCK)
                    {
                        frame->first_top_level_block_per_thread[thread_index] = frame->total_num_profile_blocks - 1;
                    }
                }
            }
        }

        if (cpu_profiler_window_open)
        {
            CPU_PROFILE_BLOCK("Display profiling");

            if (ImGui::Begin(ICON_FA_CHART_LINE "  CPU Profiling", &cpu_profiler_window_open))
            {
                static ProfilerPersistent persistent;
                ProfilerTimeline timeline(persistent, MAX_NUM_CPU_PROFILE_FRAMES);

                if (timeline.draw_header(pause_recording))
                {
                    // Recording has been stopped/resumed. Swap array into which is recorded.
                    if (pause_recording)
                    {
                        CpuProfileFrame& old_frame = profile_frames[profile_frame_write_index];
                        CpuProfileFrame& new_frame = dummy_frames[dummy_frame_write_index];
                        initialize_new_frame(old_frame, new_frame);
                    }
                    else
                    {
                        CpuProfileFrame& old_frame = dummy_frames[dummy_frame_write_index];
                        CpuProfileFrame& new_frame = profile_frames[profile_frame_write_index];
                        initialize_new_frame(old_frame, new_frame);
                    }
                }

                for (uint32 frame_index = 0; frame_index < MAX_NUM_CPU_PROFILE_FRAMES; ++frame_index)
                {
                    timeline.draw_overview_frame(profile_frames[frame_index], frame_index, current_frame);
                }
                timeline.end_overview();

                if (persistent.highlight_frame_index != -1)
                {
                    CpuProfileFrame& frame = profile_frames[persistent.highlight_frame_index];
                    ProfileBlock* blocks = profile_frames[persistent.highlight_frame_index].profile_block_pool;

                    if (frame.total_num_profile_blocks)
                    {
                        uint32 thread_indices[MAX_NUM_CPU_PROFILE_THREADS];
                        const char* thread_names[MAX_NUM_CPU_PROFILE_THREADS];
                        uint32 num_active_threads_this_frame = 0;

                        for (uint32 i = 0; i < num_threads; ++i)
                        {
                            if (frame.first_top_level_block_per_thread[i] != INVALID_PROFILE_BLOCK)
                            {
                                thread_indices[num_active_threads_this_frame] = i;
                                thread_names[num_active_threads_this_frame] = profile_thread_names[i];
                                ++num_active_threads_this_frame;
                            }
                        }

                        timeline.draw_highlight_frame_info(frame);

                        static uint32 thread_index = 0;
                        ImGui::SameLine();
                        ImGui::Dropdown("Thread", thread_names, num_active_threads_this_frame, thread_index);

                        if (persistent.highlight_frame_index != profile_frame_write_index)
                        {
                            timeline.draw_call_stack(blocks, frame.first_top_level_block_per_thread[thread_indices[thread_index]]);
                        }

                        timeline.draw_millisecond_spacings(frame);
                        timeline.handle_user_interactions();

                        if (frame.num_stats != 0)
                        {
                            ImGui::Dummy(ImVec2(0, 30.f));
                            if (ImGui::BeginChild("Stats"))
                            {
                                ImGui::Text("Frame stats");
                                ImGui::Separator();
                                for (uint32 i = 0; i < frame.num_stats; ++i)
                                {
                                    const ProfileStat& stat = frame.stats[i];
                                    switch (stat.type)
                                    {
                                        case profile_stat_type_bool:
                                            ImGui::Value(stat.label, stat.bool_value);
                                            break;
                                        case profile_stat_type_int32:
                                            ImGui::Value(stat.label, stat.int32_value);
                                            break;
                                        case profile_stat_type_uint32:
                                            ImGui::Value(stat.label, stat.uint32_value);
                                            break;
                                        case profile_stat_type_int64:
                                            ImGui::Value(stat.label, stat.int64_value);
                                            break;
                                        case profile_stat_type_uint64:
                                            ImGui::Value(stat.label, stat.uint64_value);
                                            break;
                                        case profile_stat_type_float:
                                            ImGui::Value(stat.label, stat.float_value);
                                            break;
                                        case profile_stat_type_string:
                                            ImGui::Value(stat.label, stat.string_value);
                                            break;
                                    }
                                }
                            }
                            ImGui::EndChild();
                        }
                    }
                }
            }
            ImGui::End();
        }
    }

}

#endif
