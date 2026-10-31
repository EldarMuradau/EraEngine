// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core_api.h"

#include "core/jobs/jobs.h"

#include <atomic>
#include <mutex>

namespace era_engine
{
    struct ERA_CORE_API Lock final
    {
        Lock(std::mutex& mutex)
            : sync(mutex)
        {
            sync.lock();
        }

        ~Lock()
        {
            sync.unlock();
        }

        std::mutex& sync;
    };

    // Not recursive.
    class SpinLock final
    {
        alignas(std::hardware_destructive_interference_size) std::atomic_flag flag = ATOMIC_FLAG_INIT;

    public:
        void lock()
        {
            uint32 backoff = 1;

            while (flag.test_and_set(std::memory_order_acquire))
            {
                while (flag.test(std::memory_order_relaxed))
                {
                    if (backoff <= 1024u)
                    {
                        for (uint32 i = 0; i < backoff; ++i)
                        {
                            _mm_pause();
                        }
                        backoff *= 2u;
                    }
                    else
                    {
                        JobSystem::yield();
                    }
                }
            }
        }

        bool try_lock()
        {
            return !flag.test_and_set(std::memory_order_acquire);
        }

        void unlock()
        {
            flag.clear(std::memory_order_release);
        }
    };

    class ERA_CORE_API ScopedSpinLock final
    {
        SpinLock& lock;

    public:
        ScopedSpinLock(SpinLock& _lock)
            : lock(_lock) { lock.lock(); }
        ~ScopedSpinLock() { lock.unlock(); }
    };
}