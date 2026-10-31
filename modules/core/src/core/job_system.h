#pragma once

#include "core_api.h"

#include "core/jobs/jobs.h"

#include <functional>

namespace era_engine
{
    struct JobState;

    // Handle-based jobs on top of JobSystem.
    struct ERA_CORE_API JobHandle
    {
        void submit_now();
        void submit_after(JobHandle before);
        void wait_for_completion();

        ref<JobState> state;
    };

    template <typename Data_>
    using JobFunction = void (*)(Data_&, JobHandle);

    enum class JobQueueType : uint8
    {
        // Fiber workers.
        HIGH_PRIORITY,

        // Background threads: asset I/O and other blocking work.
        LOW_PRIORITY,

        // Deferred until execute_main_thread_jobs().
        MAIN_THREAD
    };

    struct ERA_CORE_API JobQueue
    {
        explicit JobQueue(JobQueueType _type);

        template <typename Data_>
        JobHandle create_job(JobFunction<Data_> function, const Data_& data, JobHandle parent = {})
        {
            return create_job(std::function<void(JobHandle)>([function, data = Data_(data)](JobHandle job) mutable
                                  {
                                      function(data, job);
                                  }),
                parent);
        }

        JobHandle create_job(std::function<void(JobHandle)> function, JobHandle parent = {});

        JobQueueType type;
    };

    ERA_CORE_API extern JobQueue high_priority_job_queue;
    ERA_CORE_API extern JobQueue low_priority_job_queue;
    ERA_CORE_API extern JobQueue main_thread_job_queue;

    // The calling thread becomes the main thread (worker 0) of JobSystem.
    ERA_CORE_API void initialize_job_system();
    ERA_CORE_API void shutdown_job_system();

    ERA_CORE_API void execute_main_thread_jobs();

    struct DefaultJob
    {
        std::function<void()> job_func;
    };

    ERA_CORE_API JobHandle schedule_low_priority_job(const DefaultJob& job, JobHandle parent_job = {});
}
