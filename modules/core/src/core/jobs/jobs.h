// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core_api.h"

#include <atomic>
#include <vector>

namespace era_engine
{
    struct FrameParams;

    enum class JobPriority : uint8
    {
        HIGH,
        NORMAL,
        LOW,

        // Dedicated threads without fibers: blocking I/O, deep stacks, long tasks.
        BACKGROUND,

        COUNT
    };

    using JobEntryPoint = void (*)(void* data);

    struct JobDecl
    {
        JobDecl() = default;
        JobDecl(JobEntryPoint _entry_point, void* _data, JobPriority _priority = JobPriority::NORMAL, bool _main_thread = false, const char* _name = nullptr)
            : entry_point(_entry_point), data(_data), priority(_priority), main_thread(_main_thread), name(_name)
        {
        }

        JobEntryPoint entry_point = nullptr;
        void* data = nullptr;
        JobPriority priority = JobPriority::NORMAL;

        // Runs (and resumes after waits) only on worker 0 - the thread that called JobSystem::initialize().
        bool main_thread = false;

        // Shown on the worker tracks in Tracy. Must outlive the job.
        const char* name = nullptr;
    };

    // Number of unfinished jobs. Destroy it only after wait_for_counter() returned.
    class ERA_CORE_API JobCounter
    {
    public:
        JobCounter() = default;
        ~JobCounter();

        JobCounter(const JobCounter&) = delete;
        JobCounter& operator=(const JobCounter&) = delete;

        uint32 get_value() const;
        bool is_done() const;

    private:
        friend class JobSystem;
        friend struct JobCounterAccess;

        struct Waiter;

        void lock();
        void unlock();

        std::atomic<uint32> value = 0;
        std::atomic<bool> locked = false;
        Waiter* waiters = nullptr;
    };

    struct JobSystemOptions
    {
        // 0 - one worker per physical core.
        uint32 worker_count = 0;
        // Initial pool size. The pool grows when every fiber is busy or waiting.
        uint32 fiber_count = 128;
        size_t fiber_stack_reserve = 1024 * 1024;
        size_t fiber_stack_commit = 64 * 1024;
        bool pin_workers_to_cores = true;

        uint32 background_thread_count = 2;
    };

    class ERA_CORE_API JobSystem
    {
    public:
        // The calling thread becomes worker 0 and its code keeps running as the pinned main fiber.
        static bool initialize(const JobSystemOptions& options = {});

        // Must be called from the thread that called initialize(), with no unfinished jobs.
        static void shutdown();

        static bool is_initialized();

        // Before initialize() jobs run inline.
        static void run_jobs(const JobDecl* decls, uint32 count, JobCounter* counter = nullptr);

        // On a worker the current fiber is suspended while the worker keeps executing other jobs.
        // The fiber may continue on another worker. Other threads block.
        static void wait_for_counter(JobCounter* counter);

        // For work tracked outside of run_jobs(). Return the previous and the new value.
        static uint32 increment_counter(JobCounter* counter, uint32 count = 1);
        static uint32 decrement_counter(JobCounter* counter);

        // Lets other jobs run. The fiber may continue on another worker.
        static void yield();

        // Frame the current code works on. Jobs inherit it from the code that runs them (BACKGROUND jobs get none) it survives waits and moves with the fiber.
        static void set_frame_context(FrameParams* frame);
        static FrameParams* get_frame_context();

        static uint32 get_worker_count();

        static uint32 get_worker_index();
        static bool is_worker_thread();

        template <typename Func_>
        static void parallel_for(uint32 begin, uint32 end, uint32 grain_size, const Func_& func, JobPriority priority = JobPriority::NORMAL);
    };

    class ERA_CORE_API JobMutex
    {
    public:
        void lock();
        bool try_lock();
        void unlock();

    private:
        std::atomic<bool> locked = false;
    };

    template <typename Func_>
    void JobSystem::parallel_for(uint32 begin, uint32 end, uint32 grain_size, const Func_& func, JobPriority priority)
    {
        if (begin >= end)
        {
            return;
        }

        grain_size = grain_size == 0 ? 1 : grain_size;
        const uint32 job_count = (end - begin + grain_size - 1) / grain_size;

        if (job_count == 1 || !is_initialized())
        {
            for (uint32 i = begin; i < end; ++i)
            {
                func(i);
            }
            return;
        }

        struct Range
        {
            const Func_* func = nullptr;
            uint32 begin;
            uint32 end;
        };

        std::vector<Range> ranges(job_count);
        std::vector<JobDecl> decls(job_count);
        for (uint32 job = 0; job < job_count; ++job)
        {
            const uint32 range_begin = begin + job * grain_size;
            const uint32 range_end = range_begin + grain_size < end ? range_begin + grain_size : end;
            ranges[job] = Range{&func, range_begin, range_end};
            decls[job] = JobDecl([](void* data)
                {
                    const Range& range = *static_cast<const Range*>(data);
                    for (uint32 i = range.begin; i < range.end; ++i)
                    {
                        (*range.func)(i);
                    }
                },
                &ranges[job], priority, false, "parallel_for");
        }

        JobCounter counter;
        run_jobs(decls.data(), job_count, &counter);
        wait_for_counter(&counter);
    }
}
