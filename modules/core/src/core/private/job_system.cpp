#include "core/job_system.h"

namespace era_engine
{
    struct JobState
    {
        std::function<void(JobHandle)> function;
        JobQueueType type = JobQueueType::HIGH_PRIORITY;

        // Own execution + unfinished children + pending continuation registrations.
        JobCounter unfinished;

        ref<JobState> parent;
        JobHandle continuation;
    };

    namespace
    {
        std::mutex main_thread_jobs_mutex;
        std::vector<ref<JobState>> main_thread_jobs;

        void finish(const ref<JobState>& state)
        {
            if (JobSystem::decrement_counter(&state->unfinished) != 0)
            {
                return;
            }

            ref<JobState> parent = std::move(state->parent);
            JobHandle continuation = std::move(state->continuation);

            if (parent != nullptr)
            {
                finish(parent);
            }
            continuation.submit_now();
        }

        void execute(const ref<JobState>& state)
        {
            state->function(JobHandle{state});

            // Captured data is released before waiters wake up.
            state->function = nullptr;

            finish(state);
        }

        void job_entry_point(void* data)
        {
            ref<JobState>* holder = static_cast<ref<JobState>*>(data);
            ref<JobState> state = std::move(*holder);
            delete holder;

            execute(state);
        }
    }

    JobQueue::JobQueue(JobQueueType _type)
        : type(_type)
    {
    }

    JobHandle JobQueue::create_job(std::function<void(JobHandle)> function, JobHandle parent)
    {
        ref<JobState> state = make_ref<JobState>();
        state->function = std::move(function);
        state->type = type;

        JobSystem::increment_counter(&state->unfinished);

        if (parent.state != nullptr)
        {
            JobSystem::increment_counter(&parent.state->unfinished);
            state->parent = parent.state;
        }

        return JobHandle{state};
    }

    void JobHandle::submit_now()
    {
        if (state == nullptr)
        {
            return;
        }

        if (state->type == JobQueueType::MAIN_THREAD)
        {
            std::lock_guard lock(main_thread_jobs_mutex);
            main_thread_jobs.push_back(state);
            return;
        }

        const JobPriority priority = state->type == JobQueueType::HIGH_PRIORITY ? JobPriority::HIGH : JobPriority::BACKGROUND;
        const JobDecl decl(job_entry_point, new ref<JobState>(state), priority, false, "JobHandle job");
        JobSystem::run_jobs(&decl, 1);
    }

    void JobHandle::submit_after(JobHandle before)
    {
        if (before.state == nullptr)
        {
            submit_now();
            return;
        }

        // Holding a reference keeps 'before' unfinished while the continuation is attached.
        if (JobSystem::increment_counter(&before.state->unfinished) == 0)
        {
            JobSystem::decrement_counter(&before.state->unfinished);
            submit_now();
            return;
        }

        ASSERT(before.state->continuation.state == nullptr);
        before.state->continuation = *this;
        finish(before.state);
    }

    void JobHandle::wait_for_completion()
    {
        if (state == nullptr)
        {
            return;
        }

        if (state->type == JobQueueType::MAIN_THREAD && JobSystem::get_worker_index() == 0)
        {
            execute_main_thread_jobs();
        }

        JobSystem::wait_for_counter(&state->unfinished);
    }

    JobQueue high_priority_job_queue(JobQueueType::HIGH_PRIORITY);
    JobQueue low_priority_job_queue(JobQueueType::LOW_PRIORITY);
    JobQueue main_thread_job_queue(JobQueueType::MAIN_THREAD);

    void initialize_job_system()
    {
        if (!JobSystem::is_initialized())
        {
            JobSystem::initialize();
        }
    }

    void shutdown_job_system()
    {
        execute_main_thread_jobs();
        JobSystem::shutdown();
    }

    void execute_main_thread_jobs()
    {
        while (true)
        {
            std::vector<ref<JobState>> jobs;
            {
                std::lock_guard lock(main_thread_jobs_mutex);
                jobs.swap(main_thread_jobs);
            }

            if (jobs.empty())
            {
                return;
            }

            for (const ref<JobState>& state : jobs)
            {
                execute(state);
            }
        }
    }

    JobHandle schedule_low_priority_job(const DefaultJob& data, JobHandle parent_job /* = {}*/)
    {
        return low_priority_job_queue.create_job([job_func = data.job_func](JobHandle)
            {
                if (job_func)
                {
                    job_func();
                }
            },
            parent_job);
    }
}
