// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "particles/particle_systems.h"
#include "particles/fire_particle_system.h"
#include "particles/smoke_particle_system.h"
#include "particles/boid_particle_system.h"
#include "particles/debris_particle_system.h"

namespace era_engine
{
	void loadAllParticleSystemPipelines()
	{
		fire_particle_system::initializePipeline();
		smoke_particle_system::initializePipeline();
		boid_particle_system::initializePipeline();
		debris_particle_system::initializePipeline();
	}
}