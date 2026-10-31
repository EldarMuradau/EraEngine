// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#define PROFILING_INTERNAL

#include "dx/dx_profiling.h"

#include "core/imgui.h"
#include "core/math.h"

namespace era_engine
{
    bool dx_profiler_window_open = false;
}

#if ENABLE_DX_PROFILING

#include <fontawesome/IconsFontAwesome5.h>

#include <algorithm>

namespace era_engine
{
    profile_event dxProfileEvents[NUM_BUFFERED_FRAMES][MAX_NUM_DX_PROFILE_EVENTS];

#define MAX_NUM_DX_PROFILE_FRAMES 1024

    struct DXProfileFrame : profile_frame
    {
        profile_block blocks[profile_cl_count][MAX_NUM_DX_PROFILE_BLOCKS];
        uint32 count[profile_cl_count];
    };

    static DXProfileFrame profileFrames[MAX_NUM_DX_PROFILE_FRAMES];
    static uint32 profileFrameWriteIndex;
    static bool pauseRecording;

    void dxProfilingFrameEndMarker(DXCommandList* cl)
    {
        ASSERT(cl->type == D3D12_COMMAND_LIST_TYPE_DIRECT);

        uint32 queryIndex = atomicIncrement(dx_context.timestampQueryIndex[dx_context.get_buffered_frame_id()]);
        cl->query_timestamp(queryIndex);

        dxProfileEvents[dx_context.get_buffered_frame_id()][queryIndex] =
            {
                PROFILE_EVENT_FRAME_MARKER,
                profile_cl_graphics,
                getThreadIDFast(),
                "Frame End"};
    }

    void dxProfilingResolveTimeStamps(uint64* timestamps)
    {
        uint32 current_frame = profileFrameWriteIndex;

        if (!pauseRecording)
        {
            uint32 numQueries = dx_context.timestampQueryIndex[dx_context.get_buffered_frame_id()];
            if (numQueries == 0)
                return;

            profile_event* events = dxProfileEvents[dx_context.get_buffered_frame_id()];
            for (uint32 i = 0; i < numQueries; ++i)
            {
                events[i].timestamp = timestamps[i];
            }

            // Stable sort is important here, since occasionally two time stamps will have the exact same value.
            std::stable_sort(events, events + numQueries, [](const profile_event& a, const profile_event& b)
                {
                    return a.timestamp < b.timestamp;
                });


            uint16 stack[profile_cl_count][1024];
            uint32 depth[profile_cl_count] = {};
            uint32 count[profile_cl_count] = {};

            for (uint32 cl = 0; cl < profile_cl_count; ++cl)
            {
                stack[cl][0] = INVALID_PROFILE_BLOCK;
            }

            DXProfileFrame& frame = profileFrames[profileFrameWriteIndex];

            uint64 frequencies[] =
                {
                    dx_context.render_queue.time_stamp_frequency,
                    dx_context.compute_queue.time_stamp_frequency};

            for (uint32 i = 0; i < numQueries; ++i)
            {
                profile_event* e = events + i;
                ProfileCLType clType = (ProfileCLType)e->clType;

                uint64 frameEndTimestamp;
                if (handleProfileEvent(events, i, numQueries, stack[clType], depth[clType], frame.blocks[clType], count[clType], frameEndTimestamp, true))
                {
                    uint32 previousFrameIndex = (profileFrameWriteIndex == 0) ? (MAX_NUM_DX_PROFILE_FRAMES - 1) : (profileFrameWriteIndex - 1);
                    DXProfileFrame& previousFrame = profileFrames[previousFrameIndex];

                    frame.startClock = (previousFrame.endClock == 0) ? frameEndTimestamp : previousFrame.endClock;
                    frame.endClock = frameEndTimestamp;
                    frame.globalFrameID = dx_context.get_frame_id();

                    frame.duration = (float)(frame.endClock - frame.startClock) / dx_context.render_queue.time_stamp_frequency * 1000.f;

                    for (uint32 cl = 0; cl < profile_cl_count; ++cl)
                    {
                        frame.count[cl] = count[cl];

                        uint64 freq = frequencies[cl];

                        for (uint32 i = 0; i < frame.count[cl]; ++i)
                        {
                            profile_block& block = frame.blocks[cl][i];
                            block.relStart = (float)(block.startClock - frame.startClock) / freq * 1000.f;
                            block.duration = (float)(block.endClock - block.startClock) / freq * 1000.f;
                        }
                    }


                    ++profileFrameWriteIndex;
                    if (profileFrameWriteIndex >= MAX_NUM_DX_PROFILE_FRAMES)
                    {
                        profileFrameWriteIndex = 0;
                    }
                }
            }
        }

        if (dx_profiler_window_open)
        {
            if (ImGui::Begin(ICON_FA_CHART_BAR "  GPU Profiling", &dx_profiler_window_open))
            {
                static profiler_persistent persistent;
                profiler_timeline timeline(persistent, MAX_NUM_DX_PROFILE_FRAMES);

                timeline.drawHeader(pauseRecording);

                for (uint32 frame_index = 0; frame_index < MAX_NUM_DX_PROFILE_FRAMES; ++frame_index)
                {
                    timeline.drawOverviewFrame(profileFrames[frame_index], frame_index, current_frame);
                }
                timeline.endOverview();

                if (persistent.highlightFrameIndex != -1)
                {
                    uint32 colorIndex = 0;

                    DXProfileFrame& frame = profileFrames[persistent.highlightFrameIndex];

                    timeline.drawHighlightFrameInfo(frame);

                    for (uint32 clIndex = 0; clIndex < profile_cl_count; ++clIndex)
                    {
                        profile_block* blocks = frame.blocks[clIndex];
                        uint32 count = frame.count[clIndex];

                        if (count > 0)
                        {
                            timeline.drawCallStack(blocks, 0, clIndex == profile_cl_graphics ? "GRAPHICS" : "COMPUTE");
                        }
                    }

                    timeline.drawMillisecondSpacings(frame);
                    timeline.handleUserInteractions();
                }
            }
            ImGui::End();
        }
    }
}
#endif