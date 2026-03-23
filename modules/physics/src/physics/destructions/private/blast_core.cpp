#include "physics/destructions/blast_core.h"
#include "physics/core/physics.h"

#include "physics/destructions/blast_physx/NvBlastPxCallbacks.h"
#include "physics/destructions/blast_physx/NvBlastExtPxTask.h"
#include "physics/destructions/blast_physx/NvBlastPxCallbacks.h"
#include "physics/destructions/blast_physx/NvBlastExtPxManager.h"
#include "physics/destructions/blast_physx/NvBlastExtPxFamily.h"
#include "physics/destructions/blast_physx/NvBlastExtPxActor.h"
#include "physics/destructions/blast_physx/NvBlastExtPxAsset.h"

#include "NvBlast.h"
#include "NvBlastTk.h"
#include "NvBlastExtSerialization.h"
#include "NvBlastExtTkSerialization.h"

namespace era_engine::physics
{
	static physx::PxJoint* createPxJointCallback(Nv::Blast::ExtPxActor* actor0, 
		const physx::PxTransform& localFrame0, 
		Nv::Blast::ExtPxActor* actor1, 
		const physx::PxTransform& localFrame1, 
		physx::PxPhysics& physics, 
		Nv::Blast::TkJoint& joint)
	{
		using namespace physx;

		PxDistanceJoint* px_joint = PxDistanceJointCreate(physics, 
			actor0 ? &actor0->getPhysXActor() : nullptr, localFrame0,
			actor1 ? &actor1->getPhysXActor() : nullptr, localFrame1);
		px_joint->setMaxDistance(1.0f);

		return px_joint;
	}

	BlastCore::BlastCore(Physics& physics)
	{
		using namespace physx;
		using namespace Nv::Blast;

		tk_framework = NvBlastTkFrameworkCreate();

		task_manager = PxTaskManager::createTaskManager(NvBlastGetPxErrorCallback(), physics.get_cpu_dispatcher());

		TkGroupDesc group_desc{};
		group_desc.workerCount = physics.get_cpu_dispatcher()->getWorkerCount();
		tk_group = tk_framework->createGroup(group_desc);

		ext_px_manager = ExtPxManager::create(*physics.get_physics(), *tk_framework, createPxJointCallback);
		ext_px_manager->setActorCountLimit(actors_limit);

		ext_impact_damage_manager = ExtImpactDamageManager::create(ext_px_manager, ext_impact_damage_manager_settings);

		event_callback = new BlastEventCallback(ext_impact_damage_manager);

		ext_group_task_manager = ExtGroupTaskManager::create(*task_manager);
		ext_group_task_manager->setGroup(tk_group);
	}

	BlastCore::~BlastCore()
	{
		PX_RELEASE(ext_group_task_manager)
		RELEASE_PTR(event_callback)
		PX_RELEASE(ext_impact_damage_manager)
		PX_RELEASE(ext_px_manager)

		PX_RELEASE(tk_group)
		PX_RELEASE(task_manager)
		PX_RELEASE(tk_framework)
	}
}