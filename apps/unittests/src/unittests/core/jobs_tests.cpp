#include <gtest/gtest.h>

#include <core/jobs/jobs.h>
#include <core/sync.h>

#include <thread>

namespace
{
    using namespace era_engine;

    // JobSystem is initialized by the test runner's main().
    class Core_Jobs : public ::testing::Test
    {
    };

    struct NestedData
    {
        std::atomic<uint32>* leaves = nullptr;
        uint32 depth = 0;
    };

    void nested_job(void* raw)
    {
        const NestedData& data = *static_cast<const NestedData*>(raw);
        if (data.depth == 0)
        {
            data.leaves->fetch_add(1);
            return;
        }

        constexpr uint32 fan_out = 8;
        NestedData children[fan_out];
        JobDecl decls[fan_out];
        for (uint32 i = 0; i < fan_out; ++i)
        {
            children[i] = NestedData{data.leaves, data.depth - 1};
            decls[i] = JobDecl(nested_job, &children[i]);
        }

        JobCounter counter;
        JobSystem::run_jobs(decls, fan_out, &counter);
        JobSystem::wait_for_counter(&counter);
    }
}

TEST_F(Core_Jobs, RunsAllJobs)
{
    constexpr uint32 job_count = 10000;

    std::atomic<uint32> executed = 0;
    std::vector<JobDecl> decls(job_count, JobDecl([](void* data)
                                              {
                                                  static_cast<std::atomic<uint32>*>(data)->fetch_add(1);
                                              },
                                              &executed));

    JobCounter counter;
    JobSystem::run_jobs(decls.data(), job_count, &counter);
    JobSystem::wait_for_counter(&counter);

    EXPECT_EQ(executed.load(), job_count);
    EXPECT_TRUE(counter.is_done());
}

TEST_F(Core_Jobs, NestedWaitsSuspendFibers)
{
    std::atomic<uint32> leaves = 0;
    NestedData root{&leaves, 4};

    JobDecl decl(nested_job, &root);
    JobCounter counter;
    JobSystem::run_jobs(&decl, 1, &counter);
    JobSystem::wait_for_counter(&counter);

    EXPECT_EQ(leaves.load(), 8u * 8u * 8u * 8u);
}

TEST_F(Core_Jobs, WorkerIndexFollowsThreadAfterWaits)
{
    struct Data
    {
        std::mutex mutex;
        std::unordered_map<uint32, DWORD> thread_by_worker;
        std::atomic<uint32> mismatches = 0;
    } data;

    auto check = [](Data& data)
    {
        const uint32 index = JobSystem::get_worker_index();
        const DWORD thread_id = GetCurrentThreadId();

        std::lock_guard lock(data.mutex);
        auto [it, inserted] = data.thread_by_worker.emplace(index, thread_id);
        if (index >= JobSystem::get_worker_count() || it->second != thread_id)
        {
            data.mismatches.fetch_add(1);
        }
    };

    struct JobData
    {
        Data* data;
        void (*check)(Data&);
    } job_data{&data, check};

    constexpr uint32 job_count = 64;
    std::vector<JobDecl> decls(job_count, JobDecl([](void* raw)
                                              {
                                                  JobData& job = *static_cast<JobData*>(raw);
                                                  for (uint32 i = 0; i < 16; ++i)
                                                  {
                                                      job.check(*job.data);

                                                      JobDecl inner([](void*)
                                                          {
                                                              std::this_thread::yield();
                                                          },
                                                          nullptr);
                                                      JobCounter counter;
                                                      JobSystem::run_jobs(&inner, 1, &counter);
                                                      JobSystem::wait_for_counter(&counter);
                                                  }
                                                  job.check(*job.data);
                                              },
                                              &job_data));

    JobCounter counter;
    JobSystem::run_jobs(decls.data(), job_count, &counter);
    JobSystem::wait_for_counter(&counter);

    EXPECT_EQ(data.mismatches.load(), 0u);
}

TEST_F(Core_Jobs, MainThreadJobsStayOnMainThread)
{
    struct Data
    {
        DWORD main_thread_id;
        std::atomic<uint32> violations = 0;
    } data{GetCurrentThreadId()};

    constexpr uint32 job_count = 16;
    std::vector<JobDecl> decls(job_count, JobDecl([](void* raw)
                                              {
                                                  Data& data = *static_cast<Data*>(raw);
                                                  for (uint32 i = 0; i < 8; ++i)
                                                  {
                                                      if (GetCurrentThreadId() != data.main_thread_id)
                                                      {
                                                          data.violations.fetch_add(1);
                                                      }

                                                      JobDecl inner([](void*) {}, nullptr);
                                                      JobCounter counter;
                                                      JobSystem::run_jobs(&inner, 1, &counter);
                                                      JobSystem::wait_for_counter(&counter);
                                                  }
                                              },
                                              &data, JobPriority::NORMAL, true));

    JobCounter counter;
    JobSystem::run_jobs(decls.data(), job_count, &counter);
    JobSystem::wait_for_counter(&counter);

    EXPECT_EQ(GetCurrentThreadId(), data.main_thread_id);
    EXPECT_EQ(data.violations.load(), 0u);
}

TEST_F(Core_Jobs, ParallelForVisitsEveryIndexOnce)
{
    constexpr uint32 count = 100000;
    std::vector<std::atomic<uint32>> visits(count);

    JobSystem::parallel_for(0, count, 1000, [&](uint32 i)
        {
            visits[i].fetch_add(1);
        });

    uint32 wrong = 0;
    for (const auto& visit : visits)
    {
        wrong += visit.load() != 1 ? 1 : 0;
    }
    EXPECT_EQ(wrong, 0u);
}

TEST_F(Core_Jobs, JobMutexSerializesAccess)
{
    struct Data
    {
        JobMutex mutex;
        uint64 value = 0;
    } data;

    constexpr uint32 job_count = 64;
    constexpr uint32 increments = 2000;
    std::vector<JobDecl> decls(job_count, JobDecl([](void* raw)
                                              {
                                                  Data& data = *static_cast<Data*>(raw);
                                                  for (uint32 i = 0; i < increments; ++i)
                                                  {
                                                      std::lock_guard lock(data.mutex);
                                                      ++data.value;
                                                  }
                                              },
                                              &data));

    JobCounter counter;
    JobSystem::run_jobs(decls.data(), job_count, &counter);
    JobSystem::wait_for_counter(&counter);

    EXPECT_EQ(data.value, uint64(job_count) * increments);
}

TEST_F(Core_Jobs, YieldingJobsComplete)
{
    std::atomic<uint32> finished = 0;

    constexpr uint32 job_count = 32;
    std::vector<JobDecl> decls(job_count, JobDecl([](void* raw)
                                              {
                                                  for (uint32 i = 0; i < 100; ++i)
                                                  {
                                                      JobSystem::yield();
                                                  }
                                                  static_cast<std::atomic<uint32>*>(raw)->fetch_add(1);
                                              },
                                              &finished));

    JobCounter counter;
    JobSystem::run_jobs(decls.data(), job_count, &counter);
    JobSystem::wait_for_counter(&counter);

    EXPECT_EQ(finished.load(), job_count);
}

TEST_F(Core_Jobs, NonWorkerThreadWaitsBlocking)
{
    std::atomic<uint32> executed = 0;

    std::thread thread([&executed]()
        {
            EXPECT_FALSE(JobSystem::is_worker_thread());

            constexpr uint32 job_count = 256;
            std::vector<JobDecl> decls(job_count, JobDecl([](void* data)
                                                      {
                                                          static_cast<std::atomic<uint32>*>(data)->fetch_add(1);
                                                      },
                                                      &executed));

            JobCounter counter;
            JobSystem::run_jobs(decls.data(), job_count, &counter);
            JobSystem::wait_for_counter(&counter);
        });
    thread.join();

    EXPECT_EQ(executed.load(), 256u);
}

TEST_F(Core_Jobs, SpinLockSerializesWorkersAndThreads)
{
    struct Data
    {
        SpinLock lock;
        uint64 value = 0;
    } data;

    constexpr uint32 job_count = 64;
    constexpr uint32 increments = 2000;
    std::vector<JobDecl> decls(job_count, JobDecl([](void* raw)
                                              {
                                                  Data& data = *static_cast<Data*>(raw);
                                                  for (uint32 i = 0; i < increments; ++i)
                                                  {
                                                      ScopedSpinLock lock{data.lock};
                                                      ++data.value;
                                                  }
                                              },
                                              &data));

    JobCounter counter;
    JobSystem::run_jobs(decls.data(), job_count, &counter);

    std::thread thread([&data]()
        {
            for (uint32 i = 0; i < increments; ++i)
            {
                ScopedSpinLock lock{data.lock};
                ++data.value;
            }
        });

    JobSystem::wait_for_counter(&counter);
    thread.join();

    EXPECT_EQ(data.value, uint64(job_count + 1) * increments);
}

TEST(Core_SpinLock, TryLockFailsWhileHeld)
{
    SpinLock lock;
    ASSERT_TRUE(lock.try_lock());
    EXPECT_FALSE(lock.try_lock());
    lock.unlock();
    EXPECT_TRUE(lock.try_lock());
    lock.unlock();
}
