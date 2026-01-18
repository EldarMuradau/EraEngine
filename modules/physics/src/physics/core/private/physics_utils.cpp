#include "physics/core/physics_utils.h"
#include "physics/core/physics.h"
#include "physics/body_component.h"
#include "physics/cct_component.h"

#include <ecs/world.h>
#include <ecs/base_components/transform_component.h>

#include <core/log.h>
#include <core/traits.h>

namespace era_engine::physics
{
	class CharacterControllerFilterCallback final : public physx::PxQueryFilterCallback
	{
	public:
		physx::PxQueryHitType::Enum preFilter(const physx::PxFilterData& cct_filter_data, const physx::PxShape* shape, const physx::PxRigidActor*, physx::PxHitFlags&) override
		{
			using namespace physx;

			const PxFilterData& shape_filter_data = shape->getQueryFilterData();

			if ((cct_filter_data.word0 & shape_filter_data.word1) == 0 &&
				(shape_filter_data.word0 & cct_filter_data.word1) == 0)
			{
				return PxQueryHitType::eNONE;
			}
			else
			{
				return PxQueryHitType::eBLOCK;
			}
		}

		physx::PxQueryHitType::Enum postFilter(const physx::PxFilterData& filterData, const physx::PxQueryHit& hit, const physx::PxShape* shape, const physx::PxRigidActor* actor) override
		{
			using namespace physx;

			ASSERT(false);

			return PxQueryHitType::eNONE;
		}
	};

	class CCTAndCCTQueryFilterCallback final : public physx::PxControllerFilterCallback
	{
	public:
		CCTAndCCTQueryFilterCallback()
			: physx::PxControllerFilterCallback()
		{
		}

		bool filter(const physx::PxController& cct0, const physx::PxController& cct1) override
		{
			physx::PxShape* shape0 = nullptr;
			physx::PxShape* shape1 = nullptr;

			cct0.getActor()->getShapes(&shape0, 1, 0);
			ASSERT(shape0 != nullptr);

			cct1.getActor()->getShapes(&shape1, 1, 0);
			ASSERT(shape1 != nullptr);

			const physx::PxFilterData& filter_data0 = shape0->getSimulationFilterData();
			const physx::PxFilterData& filter_data1 = shape1->getSimulationFilterData();

			bool res = false;

			if ((filter_data0.word0 & filter_data1.word1) != 0 ||
				(filter_data1.word0 & filter_data0.word1) != 0)
			{
				res = true;
			}

			return res;
		}
	};

	physx::PxRigidDynamic* PhysicsUtils::create_rigid_dynamic(const physx::PxTransform& transform, void* user_data)
	{
		using namespace physx;

		PxRigidDynamic* actor = PhysicsHolder::physics_ref->get_physics()->createRigidDynamic(transform);
		ASSERT(actor != nullptr);
		
#ifndef VISUALIZE_PHYSICS
		actor->setActorFlag(PxActorFlag::eVISUALIZATION, false);
#endif

		actor->userData = user_data;

		return actor;
	}

	physx::PxRigidStatic* PhysicsUtils::create_rigid_static(const physx::PxTransform& transform, void* user_data)
	{
		using namespace physx;

		PxRigidStatic* actor = PhysicsHolder::physics_ref->get_physics()->createRigidStatic(transform);
		ASSERT(actor != nullptr);

#ifndef VISUALIZE_PHYSICS
		actor->setActorFlag(PxActorFlag::eVISUALIZATION, false);
#endif

		actor->userData = user_data;

		return actor;
	}

	BodyComponent* PhysicsUtils::get_body_component(ref<Entity::EcsData> entity_data)
	{
		return get_body_component(Entity(entity_data));
	}

	BodyComponent* PhysicsUtils::get_body_component(weakref<Entity::EcsData> entity_data)
	{
		if (entity_data.expired())
		{
			return nullptr;
		}

		return get_body_component(Entity(entity_data.lock()));
	}

	BodyComponent* PhysicsUtils::get_body_component(Entity entity)
	{
		if (DynamicBodyComponent* body_component = entity.get_component_if_exists<DynamicBodyComponent>())
		{
			return body_component;
		}
		else if (StaticBodyComponent* body_component = entity.get_component_if_exists<StaticBodyComponent>())
		{
			return body_component;
		}

		return nullptr;
	}

	void PhysicsUtils::manual_update_mass(DynamicBodyComponent* dynamic_body_component)
	{
		using namespace physx;

		if (dynamic_body_component == nullptr)
		{
			return;
		}

		if (dynamic_body_component->actor == nullptr)
		{
			return;
		}

		PxRigidDynamic* rigid_dynamic = dynamic_body_component->actor->is<PxRigidDynamic>();
		if (rigid_dynamic == nullptr)
		{
			return;
		}

		const bool body_is_kinematic = dynamic_body_component->kinematic;

		PxRigidBody* physx_rigid_body = dynamic_body_component->actor->is<PxRigidBody>();
		ASSERT(physx_rigid_body != nullptr);

		if (!body_is_kinematic)
		{
			PxVec3 center_of_mass = create_PxVec3(dynamic_body_component->center_of_mass);
			PxVec3* center_of_mass_ptr = &center_of_mass;

			const bool result = PxRigidBodyExt::setMassAndUpdateInertia(
				*physx_rigid_body,
				dynamic_body_component->mass.get(),
				center_of_mass_ptr,
				false);

			if (!result)
			{
				LOG_ERROR("Could not update mass properties for entity!");
			}
		}

		dynamic_body_component->center_of_mass.get_silent_for_write() = create_vec3(physx_rigid_body->getCMassLocalPose().p);
		dynamic_body_component->mass_space_inertia_tensor.get_silent_for_write() = create_vec3(physx_rigid_body->getMassSpaceInertiaTensor());
	}

	void PhysicsUtils::manual_set_physics_transform(Entity entity, const vec3& pos, const quat& rot, bool update_transform_component)
	{
		using namespace physx;

		BodyComponent* body_component = get_body_component(entity);

		if (body_component == nullptr)
		{
			return;
		}

		if(body_component->actor == nullptr)
		{
			return;
		}

		PxTransform physics_transform = PxTransform(create_PxVec3(pos), create_PxQuat(rot));

		PxRigidDynamic* rigid_dynamic = body_component->actor->is<PxRigidDynamic>();
		if (!body_component->actor->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION) &&
			rigid_dynamic != nullptr &&
			rigid_dynamic->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC) &&
			dynamic_cast<DynamicBodyComponent*>(body_component)->kinematic_motion_type.get() == KinematicMotionType::VELOCITY)
		{
			rigid_dynamic->setKinematicTarget(physics_transform);
		}
		else
		{
			body_component->actor->setGlobalPose(physics_transform);
		}

		if (update_transform_component)
		{
			TransformComponent* transform_component = entity.get_component<TransformComponent>();
			transform_component->set_world_transform(trs(pos, rot, transform_component->get_world_transform().scale));
		}
	}

	void PhysicsUtils::manual_set_physics_transform(Entity entity, const trs& transform, bool update_transform_component)
	{
		manual_set_physics_transform(entity, transform.position, transform.rotation, update_transform_component);
	}

	void PhysicsUtils::manual_clear_force_and_torque(DynamicBodyComponent* body_component)
	{
		using namespace physx;

		if (PxRigidDynamic* body = body_component->get_rigid_dynamic())
		{
			PxSceneWriteLock lock(*PhysicsHolder::physics_ref->get_scene());
			body->clearForce();
			body->clearTorque();

			if (body_component->kinematic)
			{
				return;
			}
			body_component->linear_velocity = vec3::zero;
			body_component->angular_velocity = vec3::zero;
		}
	}

	void PhysicsUtils::update_mass_and_inertia(DynamicBodyComponent* body_component, float density)
	{
		using namespace physx;

		if (PxRigidDynamic* body = body_component->get_rigid_dynamic())
		{
			const PxVec3 center_of_mass = create_PxVec3(body_component->center_of_mass);

			PxRigidBodyExt::updateMassAndInertia(*body, density, &center_of_mass);
		}
	}

	void PhysicsUtils::move_cct(CharacterControllerComponent* cct_component, const vec3& offset)
	{
		using namespace physx;

		World* world = cct_component->get_world();

		PxCapsuleController* physx_cct = cct_component->controller;
		ASSERT(physx_cct != nullptr);

		PxExtendedVec3 position = physx_cct->getFootPosition();

		PxRigidDynamic* physx_actor = physx_cct->getActor();
		ASSERT(physx_actor != nullptr);

		PxShape* physx_shape = nullptr;
		physx_actor->getShapes(&physx_shape, 1, 0);
		ASSERT(physx_shape != nullptr);

		PxFilterData physx_filter_data = physx_shape->getQueryFilterData();

		CharacterControllerFilterCallback cct_filter_callback;
		CCTAndCCTQueryFilterCallback cct_vs_cct_filter_callback;

		PxControllerFilters filters;
		filters.mFilterCallback = &cct_filter_callback;
		filters.mCCTFilterCallback = &cct_vs_cct_filter_callback;
		filters.mFilterData = &physx_filter_data;
		filters.mFilterFlags = PxQueryFlag::ePREFILTER | PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC;

		PxControllerCollisionFlags collision_flags = physx_cct->move(create_PxVec3(offset), 0.0f, world->get_fixed_update_dt(), filters);

		CharacterControllerCollisionFlags component_collision_flags = CharacterControllerCollisionFlags::NONE;

		if (collision_flags.isSet(PxControllerCollisionFlag::Enum::eCOLLISION_SIDES))
		{
			set_flag(component_collision_flags, CharacterControllerCollisionFlags::SIDES);
		}

		if (collision_flags.isSet(PxControllerCollisionFlag::Enum::eCOLLISION_UP))
		{
			set_flag(component_collision_flags, CharacterControllerCollisionFlags::UP);
		}

		if (collision_flags.isSet(PxControllerCollisionFlag::Enum::eCOLLISION_DOWN))
		{
			set_flag(component_collision_flags, CharacterControllerCollisionFlags::DOWN);
		}

		cct_component->current_collision_flags = component_collision_flags;

		PxExtendedVec3 new_position = physx_cct->getFootPosition();

		TransformComponent* transform_component = cct_component->get_entity().get_component<TransformComponent>();
		if (transform_component != nullptr)
		{
			transform_component->set_world_position(create_vec3(new_position));
		}
	}

}