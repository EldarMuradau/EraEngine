#pragma once

#include "core_api.h"

#include <functional>
#include <vector>

namespace era_engine
{
    // One frame on its way through the stages: world update -> render execution -> GPU.
    struct FrameParams
    {
        uint64 frame_id = 0;

        // frame_id % NUM_BUFFERED_FRAMES.
        uint32 slot = 0;

        float dt = 0.0f;

        uint64 gpu_fence = 0;

        // Execution work queued during collection, run in order.
        std::vector<std::function<void()>> render_tasks;
    };
}
