#include <gtest/gtest.h>

#include <core/job_system.h>

namespace
{
    using namespace era_engine;
}

TEST(Core_JobSystem, DefaultJobReleasesCapturedData)
{
    ref<int32> value = make_ref<int32>(0);

    DefaultJob job_object;
    job_object.job_func = [value]()
    {
        *value = 42;
    };

    JobHandle job = schedule_low_priority_job(job_object);
    job_object = {};
    job.submit_now();
    job.wait_for_completion();

    EXPECT_EQ(*value, 42);
    EXPECT_EQ(value.use_count(), 1);
}

TEST(Core_JobSystem, DefaultHandleIsNoOp)
{
    JobHandle job;
    job.submit_now();
    job.wait_for_completion();

    SUCCEED();
}

TEST(Core_JobSystem, ParentFinishesAfterChildren)
{
    struct Data
    {
        std::atomic<uint32>* finished_children;
    };

    std::atomic<uint32> finished_children = 0;

    JobHandle parent = high_priority_job_queue.create_job<Data>([](Data&, JobHandle) {}, Data{&finished_children});
    for (uint32 i = 0; i < 16; ++i)
    {
        low_priority_job_queue.create_job<Data>([](Data& data, JobHandle)
                                  {
                                      Sleep(1);
                                      data.finished_children->fetch_add(1);
                                  },
                                  Data{&finished_children}, parent)
            .submit_now();
    }
    parent.submit_now();
    parent.wait_for_completion();

    EXPECT_EQ(finished_children.load(), 16u);
}

TEST(Core_JobSystem, ContinuationRunsAfterJob)
{
    struct Data
    {
        std::atomic<uint32>* step;
        uint32 expected;
        std::atomic<bool>* ordered;
    };

    std::atomic<uint32> step = 0;
    std::atomic<bool> ordered = true;

    auto body = [](Data& data, JobHandle)
    {
        Sleep(1);
        if (data.step->fetch_add(1) != data.expected)
        {
            data.ordered->store(false);
        }
    };

    JobHandle first = low_priority_job_queue.create_job<Data>(body, Data{&step, 0, &ordered});
    JobHandle second = high_priority_job_queue.create_job<Data>(body, Data{&step, 1, &ordered});
    first.submit_now();
    second.submit_after(first);
    second.wait_for_completion();

    EXPECT_EQ(step.load(), 2u);
    EXPECT_TRUE(ordered.load());
}

TEST(Core_JobSystem, NestedBackgroundWaitsDoNotDeadlock)
{
    std::atomic<uint32> executed = 0;

    std::vector<JobHandle> outer;
    for (uint32 i = 0; i < 8; ++i)
    {
        DefaultJob job;
        job.job_func = [&executed]()
        {
            DefaultJob inner;
            inner.job_func = [&executed]()
            {
                executed.fetch_add(1);
            };
            JobHandle handle = schedule_low_priority_job(inner);
            handle.submit_now();
            handle.wait_for_completion();
        };
        outer.push_back(schedule_low_priority_job(job));
        outer.back().submit_now();
    }

    for (JobHandle& handle : outer)
    {
        handle.wait_for_completion();
    }

    EXPECT_EQ(executed.load(), 8u);
}

TEST(Core_JobSystem, MainThreadJobsRunOnMainThread)
{
    struct Data
    {
        DWORD* thread_id = nullptr;
    };

    DWORD thread_id = 0;
    JobHandle job = main_thread_job_queue.create_job<Data>([](Data& data, JobHandle)
        {
            *data.thread_id = GetCurrentThreadId();
        },
        Data{&thread_id});
    job.submit_now();
    execute_main_thread_jobs();
    job.wait_for_completion();

    EXPECT_EQ(thread_id, GetCurrentThreadId());
}
