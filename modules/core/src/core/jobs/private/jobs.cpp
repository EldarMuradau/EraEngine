// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "core/jobs/jobs.h"
#include "core/log.h"

#include <concurrentqueue/concurrentqueue.h>
#include <tracy/Tracy.hpp>
#include <tracy/TracyC.h>

#include <intrin.h>

#include <deque>
#include <memory>
#include <string>
#include <thread>

namespace era_engine
{
    struct JobFiber
    {
        void* handle = nullptr;
        const char* name = nullptr;

        // Set once the fiber's context is saved after suspension (only then it may be resumed).
        std::atomic<bool> suspended = false;

        bool main_thread = false;
        bool pooled = true;

        FrameParams* frame = nullptr;

        // Tracy connection the fiber's enter event was sent to, 0 - not entered.
        uint64 tracy_connection = 0;

        // Job the fiber is executing (it may be suspended in it).
        bool in_job = false;
        const char* job_name = nullptr;
    };

    struct JobCounter::Waiter
    {
        JobFiber* fiber = nullptr;
        HANDLE event = nullptr;
        Waiter* next = nullptr;
    };

    namespace
    {
        enum class AfterSwitch : uint8
        {
            NONE,
            RELEASE_TO_POOL,
            MARK_SUSPENDED
        };

        struct Worker
        {
            uint32 index = 0;
            void* thread_fiber = nullptr;
            std::thread thread;

            JobFiber* current = nullptr;

            // Handed over to the fiber that runs next on this worker.
            JobFiber* previous = nullptr;
            AfterSwitch after_switch = AfterSwitch::NONE;

            const char* tracy_name = nullptr;
#if defined(TRACY_ENABLE) && defined(TRACY_FIBERS)
            TracyCZoneCtx tracy_job_zone{};
#endif
            uint64 tracy_job_zone_connection = 0;
        };

        struct QueuedJob
        {
            JobDecl decl;
            JobCounter* counter = nullptr;
            FrameParams* frame = nullptr;
        };

        struct JobSystemState
        {
            JobSystemOptions options;

            std::vector<std::unique_ptr<Worker>> workers;
            std::vector<std::unique_ptr<JobFiber>> fibers;
            std::mutex fibers_mutex;

            JobFiber main_fiber;
            bool main_thread_converted = false;
            DWORD_PTR main_thread_affinity = 0;

            moodycamel::ConcurrentQueue<QueuedJob> job_queues[(size_t)JobPriority::COUNT];
            moodycamel::ConcurrentQueue<QueuedJob> main_thread_jobs;
            moodycamel::ConcurrentQueue<JobFiber*> ready_fibers;
            moodycamel::ConcurrentQueue<JobFiber*> main_thread_ready_fibers;
            moodycamel::ConcurrentQueue<JobFiber*> free_fibers;

            // Items in shared and main-thread-only queues. May be briefly negative (decremented after dequeue).
            std::atomic<int32> shared_work = 0;
            std::atomic<int32> main_thread_work = 0;

            std::atomic<uint32> sleeping_workers = 0;
            std::atomic<uint32> wake_epoch = 0;

            std::atomic<bool> running = false;

            std::vector<std::thread> background_threads;
            HANDLE background_semaphore = nullptr;
            std::atomic<bool> background_running = false;
        };

        constexpr uint32 IDLE_SPIN_COUNT = 1024;
        constexpr uint32 MUTEX_SPIN_COUNT = 256;

        JobSystemState* state = nullptr;
        std::atomic<bool> initialized = false;

        struct ThreadWaitEvent
        {
            ThreadWaitEvent()
                : handle(CreateEventW(nullptr, FALSE, FALSE, nullptr))
            {
            }

            ~ThreadWaitEvent()
            {
                CloseHandle(handle);
            }

            HANDLE handle;
        };

        thread_local Worker* tls_worker = nullptr;
        thread_local ThreadWaitEvent tls_wait_event;
        thread_local bool tls_background_thread = false;
        thread_local FrameParams* tls_frame = nullptr;

        // A fiber may continue on another thread, so the TLS address must not be cached across a switch.
        __declspec(noinline) Worker* get_current_worker()
        {
            return tls_worker;
        }

        __declspec(noinline) void set_current_worker(Worker* worker)
        {
            tls_worker = worker;
        }

        __declspec(noinline) HANDLE get_thread_wait_event()
        {
            return tls_wait_event.handle;
        }

        __declspec(noinline) bool is_background_thread()
        {
            return tls_background_thread;
        }

        const char* get_fiber_name(uint32 index)
        {
            // Tracy keeps the pointers, the names are never freed.
            static std::deque<std::string> names;
            while (names.size() <= index)
            {
                names.push_back("Job fiber " + std::to_string(names.size()));
            }
            return names[index].c_str();
        }

        const char* get_worker_name(uint32 index)
        {
            // Tracy keeps the pointers, the names are never freed.
            static std::deque<std::string> names;
            while (names.size() <= index)
            {
                names.push_back("Worker " + std::to_string(names.size()));
            }
            return names[index].c_str();
        }

        std::vector<KAFFINITY> get_physical_core_masks()
        {
            std::vector<KAFFINITY> masks;

            DWORD length = 0;
            GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &length);
            if (length == 0)
            {
                return masks;
            }

            std::vector<uint8> buffer(length);
            if (!GetLogicalProcessorInformationEx(RelationProcessorCore, (SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*)buffer.data(), &length))
            {
                return masks;
            }

            for (DWORD offset = 0; offset < length;)
            {
                const auto* entry = (const SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*)(buffer.data() + offset);
                if (entry->Relationship == RelationProcessorCore && entry->Processor.GroupMask[0].Group == 0)
                {
                    masks.push_back(entry->Processor.GroupMask[0].Mask);
                }
                offset += entry->Size;
            }

            return masks;
        }

        void wake_workers()
        {
            if (state->sleeping_workers.load() != 0)
            {
                state->wake_epoch.fetch_add(1);
                state->wake_epoch.notify_all();
            }
        }

        void push_ready(JobFiber* fiber)
        {
            if (fiber->main_thread)
            {
                state->main_thread_ready_fibers.enqueue(fiber);
                state->main_thread_work.fetch_add(1);
            }
            else
            {
                state->ready_fibers.enqueue(fiber);
                state->shared_work.fetch_add(1);
            }
            wake_workers();
        }

        bool pop_ready(const Worker* worker, JobFiber*& fiber)
        {
            if (worker->index == 0 && state->main_thread_ready_fibers.try_dequeue(fiber))
            {
                state->main_thread_work.fetch_sub(1);
                return true;
            }

            if (state->ready_fibers.try_dequeue(fiber))
            {
                state->shared_work.fetch_sub(1);
                return true;
            }

            return false;
        }

        bool pop_job(const Worker* worker, QueuedJob& job)
        {
            if (worker->index == 0 && state->main_thread_jobs.try_dequeue(job))
            {
                state->main_thread_work.fetch_sub(1);
                return true;
            }

            for (size_t priority = 0; priority < (size_t)JobPriority::BACKGROUND; ++priority)
            {
                if (state->job_queues[priority].try_dequeue(job))
                {
                    state->shared_work.fetch_sub(1);
                    return true;
                }
            }

            return false;
        }

        bool has_work(const Worker* worker)
        {
            return state->shared_work.load() > 0 || (worker->index == 0 && state->main_thread_work.load() > 0);
        }

        void idle(const Worker* worker)
        {
            for (uint32 i = 0; i < IDLE_SPIN_COUNT; ++i)
            {
                if (has_work(worker) || !state->running.load(std::memory_order_relaxed))
                {
                    return;
                }
                _mm_pause();
            }

            // Producers bump the epoch after publishing work if they see a sleeper.
            const uint32 epoch = state->wake_epoch.load();
            state->sleeping_workers.fetch_add(1);
            if (!has_work(worker) && state->running.load())
            {
                state->wake_epoch.wait(epoch);
            }
            state->sleeping_workers.fetch_sub(1);
        }

        void WINAPI fiber_entry(void* param);

        JobFiber* create_fiber()
        {
            std::lock_guard lock(state->fibers_mutex);

            auto fiber = std::make_unique<JobFiber>();
            fiber->name = get_fiber_name((uint32)state->fibers.size());
            fiber->handle = CreateFiberEx(state->options.fiber_stack_commit, state->options.fiber_stack_reserve, FIBER_FLAG_FLOAT_SWITCH, fiber_entry, fiber.get());
            ASSERT(fiber->handle != nullptr);

            state->fibers.push_back(std::move(fiber));
            return state->fibers.back().get();
        }

        // Nested waits can suspend any number of fibers, so an empty pool grows instead of blocking.
        JobFiber* acquire_free_fiber()
        {
            JobFiber* fiber = nullptr;
            if (!state->free_fibers.try_dequeue(fiber))
            {
                fiber = create_fiber();
            }
            return fiber;
        }

        void finish_switch()
        {
            Worker* worker = get_current_worker();
            JobFiber* previous = worker->previous;
            const AfterSwitch after_switch = worker->after_switch;
            worker->previous = nullptr;
            worker->after_switch = AfterSwitch::NONE;

            if (after_switch == AfterSwitch::RELEASE_TO_POOL)
            {
                state->free_fibers.enqueue(previous);
            }
            else if (after_switch == AfterSwitch::MARK_SUSPENDED)
            {
                previous->suspended.store(true, std::memory_order_release);
            }
        }

        // With TRACY_ON_DEMAND events sent before the profiler connects are dropped, so a fiber leaves only
        // the connection it entered, and enters again once a connection appears while it runs.
        void tracy_fiber_enter(JobFiber* fiber)
        {
#if defined(TRACY_ENABLE) && defined(TRACY_FIBERS)
            const tracy::Profiler& profiler = tracy::GetProfiler();
            if (profiler.IsConnected())
            {
                TracyFiberEnter(fiber->name);
                fiber->tracy_connection = profiler.ConnectionId();
                return;
            }
#endif
            fiber->tracy_connection = 0;
        }

        void tracy_fiber_leave(JobFiber* fiber)
        {
#if defined(TRACY_ENABLE) && defined(TRACY_FIBERS)
            const tracy::Profiler& profiler = tracy::GetProfiler();
            if (fiber->tracy_connection != 0 && profiler.IsConnected() && profiler.ConnectionId() == fiber->tracy_connection)
            {
                TracyFiberLeave;
            }
#endif
            fiber->tracy_connection = 0;
        }

        void tracy_fiber_ensure_entered(JobFiber* fiber)
        {
#if defined(TRACY_ENABLE) && defined(TRACY_FIBERS)
            const tracy::Profiler& profiler = tracy::GetProfiler();
            if (profiler.IsConnected() && profiler.ConnectionId() != fiber->tracy_connection)
            {
                tracy_fiber_enter(fiber);
            }
#endif
        }

        void tracy_worker_job_begin(Worker* worker, JobFiber* fiber)
        {
#if defined(TRACY_ENABLE) && defined(TRACY_FIBERS)
            static const ___tracy_source_location_data job_location = {"Job", "JobSystem worker", __FILE__, (uint32_t)__LINE__, 0};

            const tracy::Profiler& profiler = tracy::GetProfiler();
            if (!profiler.IsConnected())
            {
                return;
            }

            TracyFiberEnter(worker->tracy_name);
            worker->tracy_job_zone = ___tracy_emit_zone_begin(&job_location, 1);
            const char* name = fiber->job_name != nullptr ? fiber->job_name : "Job";
            ___tracy_emit_zone_name(worker->tracy_job_zone, name, strlen(name));
            worker->tracy_job_zone_connection = profiler.ConnectionId();

            tracy_fiber_enter(fiber);
#endif
        }

        void tracy_worker_job_end(Worker* worker, JobFiber* fiber)
        {
#if defined(TRACY_ENABLE) && defined(TRACY_FIBERS)
            const tracy::Profiler& profiler = tracy::GetProfiler();
            if (worker->tracy_job_zone_connection != 0 && profiler.IsConnected() && profiler.ConnectionId() == worker->tracy_job_zone_connection)
            {
                TracyFiberEnter(worker->tracy_name);
                ___tracy_emit_zone_end(worker->tracy_job_zone);
                tracy_fiber_enter(fiber);
            }
#endif
            worker->tracy_job_zone_connection = 0;
        }

        // Returns when self is resumed, possibly on another worker.
        void switch_to(JobFiber* self, JobFiber* target, AfterSwitch after_switch)
        {
            Worker* worker = get_current_worker();
            worker->current = target;
            worker->previous = self;
            worker->after_switch = after_switch;

            // The main fiber is profiled as its thread.
            if (self->pooled)
            {
                if (self->in_job)
                {
                    tracy_worker_job_end(worker, self);
                }
                tracy_fiber_leave(self);
            }

            SwitchToFiber(target->handle);

            if (self->pooled)
            {
                tracy_fiber_enter(self);
                if (self->in_job)
                {
                    tracy_worker_job_begin(get_current_worker(), self);
                }
            }

            finish_switch();
        }
    }

    struct JobCounterAccess
    {
        static uint32 add(JobCounter* counter, uint32 count)
        {
            return counter->value.fetch_add(count);
        }

        // False if the counter is already done.
        static bool add_waiter(JobCounter* counter, JobCounter::Waiter* waiter)
        {
            counter->lock();
            const bool pending = counter->value.load(std::memory_order_relaxed) != 0;
            if (pending)
            {
                waiter->next = counter->waiters;
                counter->waiters = waiter;
            }
            counter->unlock();
            return pending;
        }

        static uint32 finish_job(JobCounter* counter)
        {
            counter->lock();
            const uint32 left = counter->value.fetch_sub(1) - 1;
            JobCounter::Waiter* waiter = nullptr;
            if (left == 0)
            {
                waiter = counter->waiters;
                counter->waiters = nullptr;
            }
            counter->unlock();

            while (waiter != nullptr)
            {
                // A woken waiter may return immediately and release its stack.
                JobCounter::Waiter* next = waiter->next;
                JobFiber* fiber = waiter->fiber;
                HANDLE event = waiter->event;

                if (fiber != nullptr)
                {
                    push_ready(fiber);
                }
                else
                {
                    SetEvent(event);
                }

                waiter = next;
            }

            return left;
        }
    };

    namespace
    {
        void execute_job(JobFiber* self, const QueuedJob& job)
        {
            self->main_thread = job.decl.main_thread;
            self->frame = job.frame;
            self->in_job = true;
            self->job_name = job.decl.name;
            tracy_worker_job_begin(get_current_worker(), self);

            if (job.decl.entry_point != nullptr)
            {
                job.decl.entry_point(job.decl.data);
            }

            tracy_worker_job_end(get_current_worker(), self);
            self->in_job = false;
            self->job_name = nullptr;
            self->main_thread = false;
            self->frame = nullptr;

            if (job.counter != nullptr)
            {
                JobCounterAccess::finish_job(job.counter);
            }
        }

        void worker_loop(JobFiber* self)
        {
            while (state->running.load(std::memory_order_acquire))
            {
                Worker* worker = get_current_worker();

                JobFiber* ready = nullptr;
                if (pop_ready(worker, ready))
                {
                    while (!ready->suspended.load(std::memory_order_acquire))
                    {
                        _mm_pause();
                    }
                    ready->suspended.store(false, std::memory_order_relaxed);

                    switch_to(self, ready, AfterSwitch::RELEASE_TO_POOL);
                    continue;
                }

                QueuedJob job;
                if (pop_job(worker, job))
                {
                    tracy_fiber_ensure_entered(self);
                    execute_job(self, job);
                    continue;
                }

                idle(worker);
            }

            Worker* worker = get_current_worker();
            worker->current = nullptr;
            tracy_fiber_leave(self);
            SwitchToFiber(worker->thread_fiber);
        }

        void WINAPI fiber_entry(void* param)
        {
            JobFiber* self = static_cast<JobFiber*>(param);
            tracy_fiber_enter(self);
            finish_switch();
            worker_loop(self);
        }

        // Called after taking one count of the background semaphore (one count per queued job).
        void execute_background_job()
        {
            QueuedJob job;
            while (!state->job_queues[(size_t)JobPriority::BACKGROUND].try_dequeue(job))
            {
                SwitchToThread();
            }

            if (job.decl.entry_point != nullptr)
            {
                job.decl.entry_point(job.decl.data);
            }
            if (job.counter != nullptr)
            {
                JobCounterAccess::finish_job(job.counter);
            }
        }

        // A waiting background thread keeps executing background jobs, so they may be the ones it waits for.
        void wait_on_background_thread(const JobCounter* counter, HANDLE event)
        {
            const HANDLE handles[] = {event, state->background_semaphore};
            while (!counter->is_done())
            {
                const DWORD result = WaitForMultipleObjects(state->background_running.load() ? 2 : 1, handles, FALSE, INFINITE);
                if (result == WAIT_OBJECT_0)
                {
                    continue;
                }

                if (!state->background_running.load())
                {
                    // The count was released for shutdown.
                    ReleaseSemaphore(state->background_semaphore, 1, nullptr);
                    continue;
                }

                execute_background_job();
            }
        }

        void background_thread_proc(uint32 index)
        {
            SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);

            const std::wstring description = L"Background worker " + std::to_wstring(index);
            SetThreadDescription(GetCurrentThread(), description.c_str());

            tls_background_thread = true;

            while (true)
            {
                WaitForSingleObject(state->background_semaphore, INFINITE);
                if (!state->background_running.load())
                {
                    break;
                }

                execute_background_job();
            }
        }

        void worker_thread_proc(Worker* worker, KAFFINITY affinity)
        {
            if (affinity != 0)
            {
                SetThreadAffinityMask(GetCurrentThread(), affinity);
            }

            const std::wstring description = L"Job worker " + std::to_wstring(worker->index);
            SetThreadDescription(GetCurrentThread(), description.c_str());

            worker->thread_fiber = ConvertThreadToFiberEx(nullptr, FIBER_FLAG_FLOAT_SWITCH);
            ASSERT(worker->thread_fiber != nullptr);

            set_current_worker(worker);

            JobFiber* fiber = acquire_free_fiber();
            worker->current = fiber;
            SwitchToFiber(fiber->handle);

            // Back here on shutdown.
            set_current_worker(nullptr);
            ConvertFiberToThread();
        }
    }

    JobCounter::~JobCounter()
    {
        // Waits until a concurrent finish_job() releases the lock.
        lock();
        ASSERT(waiters == nullptr);
        unlock();
    }

    uint32 JobCounter::get_value() const
    {
        return value.load(std::memory_order_acquire);
    }

    bool JobCounter::is_done() const
    {
        return get_value() == 0;
    }

    void JobCounter::lock()
    {
        while (locked.exchange(true, std::memory_order_acquire))
        {
            while (locked.load(std::memory_order_relaxed))
            {
                _mm_pause();
            }
        }
    }

    void JobCounter::unlock()
    {
        locked.store(false, std::memory_order_release);
    }

    bool JobSystem::initialize(const JobSystemOptions& options)
    {
        if (initialized.load())
        {
            LOG_ERROR("Job system is already initialized.");
            return false;
        }

        const std::vector<KAFFINITY> core_masks = get_physical_core_masks();

        JobSystemOptions resolved = options;
        if (resolved.worker_count == 0)
        {
            resolved.worker_count = core_masks.empty() ? std::max(1u, std::thread::hardware_concurrency()) : (uint32)core_masks.size();
        }
        resolved.fiber_count = std::max(resolved.fiber_count, resolved.worker_count * 2);

        void* main_handle = ConvertThreadToFiberEx(nullptr, FIBER_FLAG_FLOAT_SWITCH);
        const bool converted = main_handle != nullptr;
        if (!converted)
        {
            if (GetLastError() != ERROR_ALREADY_FIBER)
            {
                LOG_ERROR("Job system: failed to convert the main thread to a fiber.");
                return false;
            }
            main_handle = GetCurrentFiber();
        }

        state = new JobSystemState();
        state->options = resolved;
        state->main_thread_converted = converted;

        state->main_fiber.handle = main_handle;
        state->main_fiber.name = "Main fiber";
        state->main_fiber.main_thread = true;
        state->main_fiber.pooled = false;

        for (uint32 i = 0; i < resolved.fiber_count; ++i)
        {
            state->free_fibers.enqueue(create_fiber());
        }

        state->workers.reserve(resolved.worker_count);
        for (uint32 i = 0; i < resolved.worker_count; ++i)
        {
            auto worker = std::make_unique<Worker>();
            worker->index = i;
            worker->tracy_name = get_worker_name(i);
            state->workers.push_back(std::move(worker));
        }

        auto get_affinity = [&](uint32 worker_index) -> KAFFINITY
        {
            return resolved.pin_workers_to_cores && !core_masks.empty() ? core_masks[worker_index % core_masks.size()] : 0;
        };

        Worker* main_worker = state->workers[0].get();
        main_worker->thread_fiber = main_handle;
        main_worker->current = &state->main_fiber;
        set_current_worker(main_worker);

        if (const KAFFINITY affinity = get_affinity(0))
        {
            state->main_thread_affinity = SetThreadAffinityMask(GetCurrentThread(), affinity);
        }

        state->running.store(true);
        initialized.store(true);

        for (uint32 i = 1; i < resolved.worker_count; ++i)
        {
            Worker* worker = state->workers[i].get();
            worker->thread = std::thread(worker_thread_proc, worker, get_affinity(i));
        }

        state->background_semaphore = CreateSemaphoreW(nullptr, 0, LONG_MAX, nullptr);
        state->background_running.store(true);
        for (uint32 i = 0; i < resolved.background_thread_count; ++i)
        {
            state->background_threads.emplace_back(background_thread_proc, i);
        }

        LOG_MESSAGE("Job system: %u workers, %u fibers, %u background threads.", resolved.worker_count, resolved.fiber_count, resolved.background_thread_count);
        return true;
    }

    void JobSystem::shutdown()
    {
        if (!initialized.load())
        {
            return;
        }

        Worker* main_worker = state->workers[0].get();
        ASSERT(get_current_worker() == main_worker && main_worker->current == &state->main_fiber);

        // Background jobs may wait for fiber jobs, so they stop first.
        state->background_running.store(false);
        ReleaseSemaphore(state->background_semaphore, (LONG)state->background_threads.size(), nullptr);
        for (std::thread& thread : state->background_threads)
        {
            thread.join();
        }
        CloseHandle(state->background_semaphore);

        state->running.store(false);
        state->wake_epoch.fetch_add(1);
        state->wake_epoch.notify_all();

        for (uint32 i = 1; i < state->workers.size(); ++i)
        {
            state->workers[i]->thread.join();
        }

        const int32 unfinished = state->shared_work.load() + state->main_thread_work.load();
        if (unfinished > 0)
        {
            LOG_WARNING("Job system: shut down with %d queued jobs or fibers.", unfinished);
        }

        for (const auto& fiber : state->fibers)
        {
            DeleteFiber(fiber->handle);
        }

        set_current_worker(nullptr);

        if (state->main_thread_converted)
        {
            ConvertFiberToThread();
        }

        if (state->main_thread_affinity != 0)
        {
            SetThreadAffinityMask(GetCurrentThread(), state->main_thread_affinity);
        }

        initialized.store(false);

        delete state;
        state = nullptr;
    }

    bool JobSystem::is_initialized()
    {
        return initialized.load(std::memory_order_acquire);
    }

    void JobSystem::run_jobs(const JobDecl* decls, uint32 count, JobCounter* counter)
    {
        if (count == 0)
        {
            return;
        }

        if (!is_initialized())
        {
            for (uint32 i = 0; i < count; ++i)
            {
                if (decls[i].entry_point != nullptr)
                {
                    decls[i].entry_point(decls[i].data);
                }
            }
            return;
        }

        if (counter != nullptr)
        {
            JobCounterAccess::add(counter, count);
        }

        FrameParams* frame = get_frame_context();

        int32 shared_count = 0;
        int32 main_thread_count = 0;
        LONG background_count = 0;
        for (uint32 i = 0; i < count; ++i)
        {
            QueuedJob job{decls[i], counter, frame};
            if (job.decl.priority == JobPriority::BACKGROUND)
            {
                job.frame = nullptr;
            }

            ASSERT(!job.decl.main_thread || job.decl.priority != JobPriority::BACKGROUND);

            if (job.decl.priority == JobPriority::BACKGROUND)
            {
                state->job_queues[(size_t)JobPriority::BACKGROUND].enqueue(job);
                ++background_count;
            }
            else if (job.decl.main_thread)
            {
                state->main_thread_jobs.enqueue(job);
                ++main_thread_count;
            }
            else
            {
                state->job_queues[(size_t)job.decl.priority].enqueue(job);
                ++shared_count;
            }
        }

        if (shared_count != 0)
        {
            state->shared_work.fetch_add(shared_count);
        }
        if (main_thread_count != 0)
        {
            state->main_thread_work.fetch_add(main_thread_count);
        }
        if (background_count != 0)
        {
            ReleaseSemaphore(state->background_semaphore, background_count, nullptr);
        }

        if (shared_count != 0 || main_thread_count != 0)
        {
            wake_workers();
        }
    }

    void JobSystem::wait_for_counter(JobCounter* counter)
    {
        if (counter == nullptr || counter->is_done())
        {
            return;
        }

        JobCounter::Waiter waiter;

        Worker* worker = get_current_worker();
        if (worker == nullptr)
        {
            waiter.event = get_thread_wait_event();
            if (JobCounterAccess::add_waiter(counter, &waiter))
            {
                if (is_background_thread())
                {
                    wait_on_background_thread(counter, waiter.event);
                }
                else
                {
                    // A signal left from an earlier wait may wake this one early.
                    while (!counter->is_done())
                    {
                        WaitForSingleObject(waiter.event, INFINITE);
                    }
                }
            }
            return;
        }

        JobFiber* self = worker->current;
        JobFiber* next = acquire_free_fiber();

        waiter.fiber = self;
        self->suspended.store(false, std::memory_order_relaxed);

        if (!JobCounterAccess::add_waiter(counter, &waiter))
        {
            state->free_fibers.enqueue(next);
            return;
        }

        switch_to(self, next, AfterSwitch::MARK_SUSPENDED);
    }

    void JobSystem::set_frame_context(FrameParams* frame)
    {
        if (Worker* worker = get_current_worker())
        {
            worker->current->frame = frame;
        }
        else
        {
            tls_frame = frame;
        }
    }

    FrameParams* JobSystem::get_frame_context()
    {
        if (const Worker* worker = get_current_worker())
        {
            return worker->current->frame;
        }
        return tls_frame;
    }

    uint32 JobSystem::increment_counter(JobCounter* counter, uint32 count)
    {
        return JobCounterAccess::add(counter, count);
    }

    uint32 JobSystem::decrement_counter(JobCounter* counter)
    {
        return JobCounterAccess::finish_job(counter);
    }

    void JobSystem::yield()
    {
        Worker* worker = get_current_worker();
        if (worker == nullptr)
        {
            SwitchToThread();
            return;
        }

        JobFiber* self = worker->current;
        JobFiber* next = acquire_free_fiber();

        self->suspended.store(false, std::memory_order_relaxed);
        push_ready(self);

        switch_to(self, next, AfterSwitch::MARK_SUSPENDED);
    }

    uint32 JobSystem::get_worker_count()
    {
        return is_initialized() ? state->options.worker_count : 0;
    }

    uint32 JobSystem::get_worker_index()
    {
        const Worker* worker = get_current_worker();
        return worker != nullptr ? worker->index : UINT32_MAX;
    }

    bool JobSystem::is_worker_thread()
    {
        return get_current_worker() != nullptr;
    }

    void JobMutex::lock()
    {
        while (true)
        {
            for (uint32 i = 0; i < MUTEX_SPIN_COUNT; ++i)
            {
                if (try_lock())
                {
                    return;
                }
                _mm_pause();
            }
            JobSystem::yield();
        }
    }

    bool JobMutex::try_lock()
    {
        return !locked.load(std::memory_order_relaxed) && !locked.exchange(true, std::memory_order_acquire);
    }

    void JobMutex::unlock()
    {
        locked.store(false, std::memory_order_release);
    }
}
