// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "physics/physx_api.h"

namespace era_engine
{
    class JobCounter;
}

namespace era_engine::physics
{
    // Runs PhysX tasks as JobSystem jobs.
    class JobCpuDispatcher final : public physx::PxCpuDispatcher
    {
    public:
        void submitTask(physx::PxBaseTask& task) override;
        uint32_t getWorkerCount() const override;
    };

    class SimulationCompletionTask final : public physx::PxLightCpuTask
    {
    public:
        void run() override;
        const char* getName() const override;

        JobCounter* counter = nullptr;
    };
}
