// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core/profiling_internal.h"
#include "core/cpu_profiling.h"

#include "dx/dx_command_list.h"

namespace era_engine
{
    extern bool dx_profiler_window_open;
}

#if ENABLE_DX_PROFILING

#if __has_include(<pix/pix3.h>)
#define USE_PIX
#include <pix/pix3.h>
#pragma comment(lib, "WinPixEventRuntime")
#else
#define PIXScopedEvent(...)
#define PIX_COLOR(...)
#endif

#define _DX_PROFILE_BLOCK_(counter, cl, name) era_engine::DXProfileBlockRecorder COMPOSITE_VARNAME(__DX_PROFILE_BLOCK, counter)(cl, name)
#define DX_PROFILE_BLOCK(cl, name)             \
    _DX_PROFILE_BLOCK_(__COUNTER__, cl, name); \
    PIXScopedEvent(cl->command_list.Get(), PIX_COLOR(255, 255, 0), name);

#define MAX_NUM_DX_PROFILE_BLOCKS 2048
#define MAX_NUM_DX_PROFILE_EVENTS (MAX_NUM_DX_PROFILE_BLOCKS * 2) // One for start and end.

namespace era_engine
{
    enum ProfileCLType
    {
        profile_cl_graphics,
        profile_cl_compute,

        profile_cl_count,
    };

    extern profile_event dxProfileEvents[NUM_BUFFERED_FRAMES][MAX_NUM_DX_PROFILE_EVENTS];

    struct DXProfileBlockRecorder
    {
        DXCommandList* cl;
        const char* name;

        DXProfileBlockRecorder(DXCommandList* cl, const char* name)
            : cl(cl), name(name)
        {
            uint32 queryIndex = atomicIncrement(dx_context.timestampQueryIndex[dx_context.get_buffered_frame_id()]);
            cl->query_timestamp(queryIndex);

            dxProfileEvents[dx_context.get_buffered_frame_id()][queryIndex] =
                {
                    PROFILE_EVENT_BEGIN_BLOCK,
                    (uint16)((cl->type == D3D12_COMMAND_LIST_TYPE_DIRECT) ? profile_cl_graphics : profile_cl_compute),
                    getThreadIDFast(),
                    name};
        }

        ~DXProfileBlockRecorder()
        {
            uint32 queryIndex = atomicIncrement(dx_context.timestampQueryIndex[dx_context.get_buffered_frame_id()]);
            cl->query_timestamp(queryIndex);

            dxProfileEvents[dx_context.get_buffered_frame_id()][queryIndex] =
                {
                    PROFILE_EVENT_END_BLOCK,
                    (uint16)((cl->type == D3D12_COMMAND_LIST_TYPE_DIRECT) ? profile_cl_graphics : profile_cl_compute),
                    getThreadIDFast(),
                    name};
        }
    };

    void dxProfilingFrameEndMarker(DXCommandList* cl);
    void dxProfilingResolveTimeStamps(uint64* timestamps);
}
#else

#define DX_PROFILE_BLOCK(...)

#define dxProfilingFrameEndMarker(...)
#define dxProfilingResolveTimeStamps(...)

#endif

#define PROFILE_ALL(cl, name)   \
    DX_PROFILE_BLOCK(cl, name); \
    CPU_PROFILE_BLOCK(name)