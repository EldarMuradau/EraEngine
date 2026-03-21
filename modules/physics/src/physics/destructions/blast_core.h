#pragma once

#include "physics_api.h"
#include "physics/physx_api.h"

#include "physics/destructions/blast_physx/NvBlastExtPxManager.h"
#include "physics/destructions/blast_physx/NvBlastExtImpactDamageManager.h"

#include <stress/NvBlastExtStressSolver.h>

namespace Nv::Blast
{
	class TkFramework;
	class TkGroup;

	class ExtGroupTaskManager;
}

namespace era_engine::physics
{
	class BlastEventCallback : public physx::PxSimulationEventCallback
	{
	public:
		BlastEventCallback(Nv::Blast::ExtImpactDamageManager* _manager) : manager(_manager) {}

		void onContact(const physx::PxContactPairHeader& pair_header, const physx::PxContactPair* pairs, uint32 nb_pairs) override
		{
			manager->onContact(pair_header, pairs, nb_pairs);
		}

	private:
		void onConstraintBreak(physx::PxConstraintInfo*, physx::PxU32) override {}
		void onWake(physx::PxActor**, physx::PxU32) override {}
		void onSleep(physx::PxActor**, physx::PxU32) override {}
		void onTrigger(physx::PxTriggerPair*, physx::PxU32) override {}
		void onAdvance(const physx::PxRigidBody* const*, const physx::PxTransform*, const physx::PxU32) override {}

		Nv::Blast::ExtImpactDamageManager* manager = nullptr;
	};

	class ERA_PHYSICS_API BlastCore final
	{
	public:
		BlastCore();
		~BlastCore();

		uint32 actors_limit = 100000;

		BlastEventCallback* event_callback = nullptr;

		Nv::Blast::TkFramework* tk_framework = nullptr;
		physx::PxTaskManager* task_manager = nullptr;

		Nv::Blast::TkGroup* tk_group = nullptr;
		Nv::Blast::ExtGroupTaskManager* ext_group_task_manager = nullptr;

		Nv::Blast::ExtPxManager* ext_px_manager = nullptr;

		Nv::Blast::ExtImpactDamageManager* ext_impact_damage_manager = nullptr;
		Nv::Blast::ExtImpactSettings ext_impact_damage_manager_settings;
		Nv::Blast::ExtStressSolverSettings ext_stress_solver_settings;
	};
}