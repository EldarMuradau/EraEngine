#include "physics/articulation.h"

#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		
		registration::enumeration<ArticulationCacheFlags>("ArticulationCacheFlags")
		(
			value("NONE", ArticulationCacheFlags::NONE),
			value("VELOCITY", ArticulationCacheFlags::VELOCITY),
			value("ACCELERATION", ArticulationCacheFlags::ACCELERATION),
			value("POSITION", ArticulationCacheFlags::POSITION),
			value("FORCE", ArticulationCacheFlags::FORCE),
			value("LINK_VELOCITY", ArticulationCacheFlags::LINK_VELOCITY),
			value("LINK_ACCELERATION", ArticulationCacheFlags::LINK_ACCELERATION),
			value("ROOT_TRANSFORM", ArticulationCacheFlags::ROOT_TRANSFORM),
			value("ROOT_VELOCITIES", ArticulationCacheFlags::ROOT_VELOCITIES),
			value("ALL", ArticulationCacheFlags::ALL)
		);

		registration::class_<ArticulationComponent>("ArticulationComponent")
			.constructor<>();
	}

	ArticulationComponent::ArticulationComponent(ref<Entity::EcsData> _data, const ArticulationComponentDescriptor& _descriptor)
		: Component(_data), descriptor(_descriptor)
	{
		using namespace physx;

		PxPhysics* physics = PhysicsEngine::get_physics_core()->get_physics();

		PhysicsEngine::execute_write([&]() {
			articulation = physics->createArticulationReducedCoordinate();

			articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, descriptor.fixed_base);
			articulation->setArticulationFlag(PxArticulationFlag::eDISABLE_SELF_COLLISION, !descriptor.self_collision);

			PxTransform rootTrs = PxTransform(create_PxVec3(descriptor.root_position));

			articulation->setRootGlobalPose(rootTrs);

			PxArticulationLink* link = articulation->createLink(nullptr, rootTrs);

			// Test data
			{
				PxSphereGeometry geometry = PxSphereGeometry(1.0f);

				physx::PxRigidActorExt::createExclusiveShape(*link, geometry, *PhysicsEngine::get_physics_core()->get_default_material()->get_native_material());
				physx::PxRigidBodyExt::updateMassAndInertia(*link, 1.0f);

				PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
				joint->setParentPose(rootTrs);
				joint->setChildPose(rootTrs);

				joint->setJointType(PxArticulationJointType::eREVOLUTE);
				joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eLIMITED);
				PxArticulationLimit limits;
				limits.low = -PxPiDivFour;
				limits.high = PxPiDivFour;
				joint->setLimitParams(PxArticulationAxis::eSWING2, limits);

				PxArticulationDrive posDrive;
				posDrive.stiffness = 0.5f;
				posDrive.damping = 0.05f;
				posDrive.maxForce = 100.0f;
				posDrive.driveType = PxArticulationDriveType::eFORCE;

				joint->setDriveParams(PxArticulationAxis::eSWING2, posDrive);
				joint->setDriveVelocity(PxArticulationAxis::eSWING2, 0.0f);
				joint->setDriveTarget(PxArticulationAxis::eSWING2, 1.0f);
			}

			PhysicsEngine::get_physics_core()->get_scene()->addArticulation(*articulation);
		});
	}

	ArticulationComponent::~ArticulationComponent()
	{
		PhysicsEngine::get_physics_core()->get_scene()->removeArticulation(*articulation);

		joints.clear();
		links.clear();

		PX_RELEASE(cache)

		PX_RELEASE(articulation)
	}

	void ArticulationComponent::apply_cache(ArticulationCacheFlags flags, physx::PxArticulationCache* in_cache)
	{
		using namespace physx;
		if (in_cache)
		{
			articulation->applyCache(*in_cache, (PxArticulationCacheFlags)((uint32)flags));
		}
		else
		{
			articulation->applyCache(*cache, (PxArticulationCacheFlags)((uint32)flags));
		}
	}

	physx::PxArticulationCache* ArticulationComponent::create_cache(ArticulationCacheFlags flags)
	{
		using namespace physx;

		PX_RELEASE(cache)

		cache = articulation->createCache();
		articulation->copyInternalStateToCache(*cache, (PxArticulationCacheFlags)((uint32)flags));
		return cache;
	}

}