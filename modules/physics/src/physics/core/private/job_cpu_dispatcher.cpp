#include "physics/core/job_cpu_dispatcher.h"

#include <core/cpu_profiling.h>
#include <core/jobs/jobs.h>

namespace era_engine::physics
{
    void JobCpuDispatcher::submitTask(physx::PxBaseTask& task)
    {
        const JobDecl decl([](void* data)
            {
                physx::PxBaseTask& task = *static_cast<physx::PxBaseTask*>(data);

                ZoneScopedN("PhysX task");
                const char* name = task.getName();
                ZoneText(name, strlen(name));

                task.run();
                task.release();
            },
            &task, task.isHighPriority() ? JobPriority::HIGH : JobPriority::NORMAL, false, task.getName());

        JobSystem::run_jobs(&decl, 1);
    }

    uint32_t JobCpuDispatcher::getWorkerCount() const
    {
        return std::max(1u, JobSystem::get_worker_count());
    }

    void SimulationCompletionTask::run()
    {
        JobSystem::decrement_counter(counter);
    }

    const char* SimulationCompletionTask::getName() const
    {
        return "Era.SimulationCompletion";
    }
}
