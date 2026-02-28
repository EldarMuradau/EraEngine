#include "physics/core/joints_system.h"
#include "physics/core/physics.h"
#include "physics/body_component.h"
#include "physics/joint.h"
#include "physics/core/physics_utils.h"
#include "physics/shape_utils.h"

#include <core/cpu_profiling.h>

#include <ecs/base_components/transform_component.h>
#include <ecs/update_groups.h>

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<JointsSystem>("JointsSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("physics")))
			.method("update", &JointsSystem::update)
			(metadata("update_group", update_types::PHYSICS), 
			 metadata("After", std::vector<std::string>{"PhysicsSystem::update"}));
	}

	JointsSystem::JointsSystem(World* _world)
		: System(_world)
	{
	}

	void JointsSystem::init()
	{
		entt::registry& registry = world->get_registry();
		registry.on_construct<D6JointComponent>().connect<&JointsSystem::on_d6_joint_created>(this);
		registry.on_construct<FixedJointComponent>().connect<&JointsSystem::on_fixed_joint_created>(this);
		registry.on_construct<DistanceJointComponent>().connect<&JointsSystem::on_distance_joint_created>(this);
	}

	void JointsSystem::update(float dt)
	{
		ZoneScopedN("JointsSystem::update");

		process_added_joints();

		sync_physics_to_component_changes();
	}

	void JointsSystem::sync_physics_to_component_changes()
	{
		using namespace physx;

		for (auto [entity_handle, observable_component, joint_component] : world->group(components_group<TransformComponent, FixedJointComponent>).each())
		{
			PxJoint* joint = joint_component.get_native_joint();

			if (joint == nullptr)
			{
				continue;
			}

			PxFixedJoint* native_joint = joint->is<PxFixedJoint>();

			if (native_joint == nullptr)
			{
				continue;
			}

			if (joint_component.disabled.is_changed())
			{
				if (joint_component.disabled)
				{
					native_joint->setActors(PhysicsUtils::get_body_component(joint_component.get_first_entity_ptr().get())->get_rigid_actor(), nullptr);
				}
				else
				{
					native_joint->setActors(PhysicsUtils::get_body_component(joint_component.get_first_entity_ptr().get())->get_rigid_actor(),
						PhysicsUtils::get_body_component(joint_component.get_second_entity_ptr().get())->get_rigid_actor());
				}
				native_joint->setConstraintFlag(PxConstraintFlag::eDISABLE_CONSTRAINT, joint_component.disabled);
				joint_component.disabled.sync_changes();
			}

			if (joint_component.enable_collision.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, joint_component.enable_collision);
				joint_component.enable_collision.sync_changes();
			}

			if (joint_component.break_force.is_changed())
			{
				native_joint->setBreakForce(joint_component.break_force.get(), joint_component.break_force.get() * 2.0f);
				joint_component.break_force.sync_changes();
			}

			if (joint_component.always_update.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eALWAYS_UPDATE, joint_component.always_update);
				joint_component.always_update.sync_changes();
			}

			if (joint_component.disable_preprocessing.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, joint_component.disable_preprocessing);
				joint_component.disable_preprocessing.sync_changes();
			}
		}

		for (auto [entity_handle, observable_component, joint_component] : world->group(components_group<TransformComponent, RevoluteJointComponent>).each())
		{
			PxJoint* joint = joint_component.get_native_joint();

			if (joint == nullptr)
			{
				continue;
			}

			PxRevoluteJoint* native_joint = joint->is<PxRevoluteJoint>();

			if (native_joint == nullptr)
			{
				continue;
			}

			if (joint_component.disabled.is_changed())
			{
				if (joint_component.disabled)
				{
					native_joint->setActors(PhysicsUtils::get_body_component(joint_component.get_first_entity_ptr().get())->get_rigid_actor(), nullptr);
				}
				else
				{
					native_joint->setActors(PhysicsUtils::get_body_component(joint_component.get_first_entity_ptr().get())->get_rigid_actor(),
						PhysicsUtils::get_body_component(joint_component.get_second_entity_ptr().get())->get_rigid_actor());
				}
				native_joint->setConstraintFlag(PxConstraintFlag::eDISABLE_CONSTRAINT, joint_component.disabled);
				joint_component.disabled.sync_changes();
			}

			if (joint_component.enable_collision.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, joint_component.enable_collision);
				joint_component.enable_collision.sync_changes();
			}

			if (joint_component.break_force.is_changed())
			{
				native_joint->setBreakForce(joint_component.break_force.get(), joint_component.break_force.get() * 2.0f);
				joint_component.break_force.sync_changes();
			}

			if (joint_component.always_update.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eALWAYS_UPDATE, joint_component.always_update);
				joint_component.always_update.sync_changes();
			}

			if (joint_component.enable_drive.is_changed())
			{
				native_joint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, joint_component.enable_drive);
				joint_component.enable_drive.sync_changes();
			}

			if (joint_component.drive_force_limit.is_changed())
			{
				native_joint->setDriveForceLimit(joint_component.drive_force_limit);
				joint_component.drive_force_limit.sync_changes();
			}

			if (joint_component.drive_gear_ratio.is_changed())
			{
				native_joint->setDriveGearRatio(joint_component.drive_gear_ratio);
				joint_component.drive_gear_ratio.sync_changes();
			}

			if (joint_component.drive_velocity.is_changed())
			{
				native_joint->setDriveVelocity(joint_component.drive_velocity);
				joint_component.drive_velocity.sync_changes();
			}

			if (joint_component.linear_limit.is_changed())
			{
				native_joint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
				native_joint->setLimit(PxJointAngularLimitPair{ joint_component.linear_limit->x, joint_component.linear_limit->y });
				joint_component.linear_limit.sync_changes();
			}

			if (joint_component.disable_preprocessing.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, joint_component.disable_preprocessing);
				joint_component.disable_preprocessing.sync_changes();
			}
		}

		for (auto [entity_handle, observable_component, joint_component] : world->group(components_group<TransformComponent, SphericalJointComponent>).each())
		{
			PxJoint* joint = joint_component.get_native_joint();

			if (joint == nullptr)
			{
				continue;
			}

			PxSphericalJoint* native_joint = joint->is<PxSphericalJoint>();

			if (native_joint == nullptr)
			{
				continue;
			}

			if (joint_component.disabled.is_changed())
			{
				if (joint_component.disabled)
				{
					native_joint->setActors(PhysicsUtils::get_body_component(joint_component.get_first_entity_ptr().get())->get_rigid_actor(), nullptr);
				}
				else
				{
					native_joint->setActors(PhysicsUtils::get_body_component(joint_component.get_first_entity_ptr().get())->get_rigid_actor(),
						PhysicsUtils::get_body_component(joint_component.get_second_entity_ptr().get())->get_rigid_actor());
				}
				native_joint->setConstraintFlag(PxConstraintFlag::eDISABLE_CONSTRAINT, joint_component.disabled);
				joint_component.disabled.sync_changes();
			}

			if (joint_component.enable_collision.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, joint_component.enable_collision);
				joint_component.enable_collision.sync_changes();
			}

			if (joint_component.break_force.is_changed())
			{
				native_joint->setBreakForce(joint_component.break_force.get(), joint_component.break_force.get() * 2.0f);
				joint_component.break_force.sync_changes();
			}

			if (joint_component.always_update.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eALWAYS_UPDATE, joint_component.always_update);
				joint_component.always_update.sync_changes();
			}

			if (joint_component.angular_limit.is_changed())
			{
				native_joint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
				native_joint->setLimitCone(PxJointLimitCone{ joint_component.angular_limit->x, joint_component.angular_limit->y });
				joint_component.angular_limit.sync_changes();
			}

			if (joint_component.disable_preprocessing.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, joint_component.disable_preprocessing);
				joint_component.disable_preprocessing.sync_changes();
			}
		}

		for (auto [entity_handle, observable_component, joint_component] : world->group(components_group<TransformComponent, DistanceJointComponent>).each())
		{
			PxJoint* joint = joint_component.get_native_joint();

			if (joint == nullptr)
			{
				continue;
			}

			PxDistanceJoint* native_joint = joint->is<PxDistanceJoint>();

			if (native_joint == nullptr)
			{
				continue;
			}
			
			if (joint_component.stiffness.is_changed())
			{
				native_joint->setStiffness(joint_component.stiffness);
				joint_component.stiffness.sync_changes();
			}

			if (joint_component.damping.is_changed())
			{
				native_joint->setDamping(joint_component.damping);
				joint_component.damping.sync_changes();
			}

			if (joint_component.spring_enabled.is_changed())
			{
				native_joint->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, joint_component.spring_enabled);
				joint_component.spring_enabled.sync_changes();
			}

			if (joint_component.min_distance.is_changed())
			{
				native_joint->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, true);
				native_joint->setMinDistance(joint_component.min_distance);
				joint_component.min_distance.sync_changes();
			}

			if (joint_component.max_distance.is_changed())
			{
				native_joint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
				native_joint->setMaxDistance(joint_component.max_distance);
				joint_component.max_distance.sync_changes();
			}

			if (joint_component.disabled.is_changed())
			{
				if (joint_component.disabled)
				{
					native_joint->setActors(PhysicsUtils::get_body_component(joint_component.get_first_entity_ptr().get())->get_rigid_actor(), nullptr);
				}
				else
				{
					native_joint->setActors(PhysicsUtils::get_body_component(joint_component.get_first_entity_ptr().get())->get_rigid_actor(),
						PhysicsUtils::get_body_component(joint_component.get_second_entity_ptr().get())->get_rigid_actor());
				}
				native_joint->setConstraintFlag(PxConstraintFlag::eDISABLE_CONSTRAINT, joint_component.disabled);
				joint_component.disabled.sync_changes();
			}

			if (joint_component.enable_collision.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, joint_component.enable_collision);
				joint_component.enable_collision.sync_changes();
			}

			if (joint_component.break_force.is_changed())
			{
				native_joint->setBreakForce(joint_component.break_force.get(), joint_component.break_force.get() * 2.0f);
				joint_component.break_force.sync_changes();
			}

			if (joint_component.always_update.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eALWAYS_UPDATE, joint_component.always_update);
				joint_component.always_update.sync_changes();
			}

			if (joint_component.disable_preprocessing.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, joint_component.disable_preprocessing);
				joint_component.disable_preprocessing.sync_changes();
			}
		}

		for (auto [entity_handle, observable_component, joint_component] : world->group(components_group<TransformComponent, D6JointComponent>).each())
		{
			PxJoint* joint = joint_component.get_native_joint();

			if (joint == nullptr)
			{
				continue;
			}

			PxD6Joint* native_joint = joint->is<PxD6Joint>();

			if (native_joint == nullptr)
			{
				continue;
			}

			if (joint_component.disabled.is_changed())
			{
				if (joint_component.disabled)
				{
					native_joint->setActors(PhysicsUtils::get_body_component(joint_component.get_first_entity_ptr().get())->get_rigid_actor(), nullptr);
				}
				else
				{
					native_joint->setActors(PhysicsUtils::get_body_component(joint_component.get_first_entity_ptr().get())->get_rigid_actor(), 
						PhysicsUtils::get_body_component(joint_component.get_second_entity_ptr().get())->get_rigid_actor());
				}
				native_joint->setConstraintFlag(PxConstraintFlag::eDISABLE_CONSTRAINT, joint_component.disabled);
				joint_component.disabled.sync_changes();
			}

			if (joint_component.enable_collision.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, joint_component.enable_collision);
				joint_component.enable_collision.sync_changes();
			}

			if (joint_component.break_force.is_changed())
			{
				native_joint->setBreakForce(joint_component.break_force.get(), joint_component.break_force.get() * 2.0f);
				joint_component.break_force.sync_changes();
			}

			if (joint_component.always_update.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eALWAYS_UPDATE, joint_component.always_update);
				joint_component.always_update.sync_changes();
			}

			if (joint_component.swing_y_limit.is_changed() ||
				joint_component.swing_z_limit.is_changed() ||
				joint_component.swing_limit_stiffness.is_changed() ||
				joint_component.swing_limit_damping.is_changed() ||
				joint_component.swing_limit_restitution.is_changed())
			{
				PxJointLimitCone limit(
					joint_component.swing_y_limit,
					joint_component.swing_z_limit,
					PxSpring(joint_component.swing_limit_stiffness, joint_component.swing_limit_damping));
				limit.restitution = joint_component.swing_limit_restitution;
				native_joint->setSwingLimit(limit);

				joint_component.swing_y_limit.sync_changes();
				joint_component.swing_z_limit.sync_changes();
				joint_component.swing_limit_stiffness.sync_changes();
				joint_component.swing_limit_damping.sync_changes();
				joint_component.swing_limit_restitution.sync_changes();
			}

			if (joint_component.twist_min_limit.is_changed() ||
				joint_component.twist_max_limit.is_changed() ||
				joint_component.twist_limit_stiffness.is_changed() ||
				joint_component.twist_limit_damping.is_changed() ||
				joint_component.twist_limit_restitution.is_changed())
			{
				PxJointAngularLimitPair limit(
					joint_component.twist_min_limit,
					joint_component.twist_max_limit,
					PxSpring(joint_component.twist_limit_stiffness, joint_component.twist_limit_damping));
				limit.restitution = joint_component.twist_limit_restitution;

				native_joint->setTwistLimit(limit);

				joint_component.twist_min_limit.sync_changes();
				joint_component.twist_max_limit.sync_changes();
				joint_component.twist_limit_stiffness.sync_changes();
				joint_component.twist_limit_damping.sync_changes();
				joint_component.twist_limit_restitution.sync_changes();
			}

			if (joint_component.linear_limit.is_changed())
			{
				native_joint->setLinearLimit(PxJointLinearLimit{ joint_component.linear_limit });
				joint_component.linear_limit.sync_changes();
			}

			if (joint_component.distance_limit.is_changed())
			{
				native_joint->setDistanceLimit(PxJointLinearLimit{ joint_component.distance_limit });
				joint_component.distance_limit.sync_changes();
			}

			if (joint_component.swing_y_motion_type.is_changed())
			{
				native_joint->setMotion(PxD6Axis::eSWING1, static_cast<PxD6Motion::Enum>(joint_component.swing_y_motion_type.get()));
				joint_component.swing_y_motion_type.sync_changes();
			}
			if (joint_component.swing_z_motion_type.is_changed())
			{
				native_joint->setMotion(PxD6Axis::eSWING2, static_cast<PxD6Motion::Enum>(joint_component.swing_z_motion_type.get()));
				joint_component.swing_z_motion_type.sync_changes();
			}
			if (joint_component.twist_motion_type.is_changed())
			{
				native_joint->setMotion(PxD6Axis::eTWIST, static_cast<PxD6Motion::Enum>(joint_component.twist_motion_type.get()));
				joint_component.twist_motion_type.sync_changes();
			}
			if (joint_component.linear_x_motion_type.is_changed())
			{
				native_joint->setMotion(PxD6Axis::eX, static_cast<PxD6Motion::Enum>(joint_component.linear_x_motion_type.get()));
				joint_component.linear_x_motion_type.sync_changes();
			}
			if (joint_component.linear_y_motion_type.is_changed())
			{
				native_joint->setMotion(PxD6Axis::eY, static_cast<PxD6Motion::Enum>(joint_component.linear_y_motion_type.get()));
				joint_component.linear_y_motion_type.sync_changes();
			}
			if (joint_component.linear_z_motion_type.is_changed())
			{
				native_joint->setMotion(PxD6Axis::eZ, static_cast<PxD6Motion::Enum>(joint_component.linear_z_motion_type.get()));
				joint_component.linear_z_motion_type.sync_changes();
			}

			if (joint_component.drive_limits_are_forces.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, joint_component.drive_limits_are_forces);
				joint_component.drive_limits_are_forces.sync_changes();
			}

			if (joint_component.improved_slerp.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eIMPROVED_SLERP, joint_component.improved_slerp);
				joint_component.improved_slerp.sync_changes();
			}

			if (joint_component.disable_preprocessing.is_changed())
			{
				native_joint->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, joint_component.disable_preprocessing);
				joint_component.disable_preprocessing.sync_changes();
			}

			if ((joint_component.twist_drive_accelerated.is_changed() ||
				joint_component.twist_drive_damping.is_changed() ||
				joint_component.twist_drive_stiffness.is_changed() ||
				joint_component.twist_drive_force_limit.is_changed()) &&
				!joint_component.perform_slerp_drive)
			{
				PxD6JointDrive drive;
				drive.stiffness = joint_component.twist_drive_stiffness;
				drive.damping = joint_component.twist_drive_damping;
				drive.forceLimit = joint_component.twist_drive_force_limit;
				drive.flags = static_cast<PxD6JointDriveFlags>(joint_component.twist_drive_accelerated ? 1 : 0);
				native_joint->setDrive(PxD6Drive::eTWIST, drive);

				PxRigidActor* actor0 = nullptr;
				PxRigidActor* actor1 = nullptr;
				native_joint->getActors(actor0, actor1);

				if (PxRigidDynamic* actor_dynamic = actor0 == nullptr ? nullptr : actor0->is<PxRigidDynamic>(); actor_dynamic != nullptr)
				{
					if (!actor_dynamic->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION) &&
						!actor_dynamic->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
					{
						actor_dynamic->wakeUp();
					}
				}

				if (PxRigidDynamic* actor_dynamic = actor1 == nullptr ? nullptr : actor1->is<PxRigidDynamic>(); actor_dynamic != nullptr)
				{
					if (!actor_dynamic->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION) &&
						!actor_dynamic->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
					{
						actor_dynamic->wakeUp();
					}
				}

				joint_component.twist_drive_damping.sync_changes();
				joint_component.twist_drive_accelerated.sync_changes();
				joint_component.twist_drive_stiffness.sync_changes();
				joint_component.twist_drive_force_limit.sync_changes();
			}

			if ((joint_component.swing_drive_accelerated.is_changed() ||
				joint_component.swing_drive_damping.is_changed() ||
				joint_component.swing_drive_stiffness.is_changed() ||
				joint_component.swing_drive_force_limit.is_changed()) &&
				!joint_component.perform_slerp_drive)
			{
				PxD6JointDrive drive;
				drive.stiffness = joint_component.swing_drive_stiffness;
				drive.damping = joint_component.swing_drive_damping;
				drive.forceLimit = joint_component.swing_drive_force_limit;
				drive.flags = static_cast<PxD6JointDriveFlags>(joint_component.swing_drive_accelerated ? 1 : 0);
				native_joint->setDrive(PxD6Drive::eSWING, drive);

				PxRigidActor* actor0 = nullptr;
				PxRigidActor* actor1 = nullptr;
				native_joint->getActors(actor0, actor1);

				if (PxRigidDynamic* actor_dynamic = actor0 == nullptr ? nullptr : actor0->is<PxRigidDynamic>(); actor_dynamic != nullptr)
				{
					if (!actor_dynamic->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION) &&
						!actor_dynamic->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
					{
						actor_dynamic->wakeUp();
					}
				}

				if (PxRigidDynamic* actor_dynamic = actor1 == nullptr ? nullptr : actor1->is<PxRigidDynamic>(); actor_dynamic != nullptr)
				{
					if (!actor_dynamic->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION) &&
						!actor_dynamic->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
					{
						actor_dynamic->wakeUp();
					}
				}

				joint_component.swing_drive_damping.sync_changes();
				joint_component.swing_drive_accelerated.sync_changes();
				joint_component.swing_drive_stiffness.sync_changes();
				joint_component.swing_drive_force_limit.sync_changes();
			}

			if ((joint_component.slerp_drive_accelerated.is_changed() ||
				joint_component.slerp_drive_damping.is_changed() ||
				joint_component.slerp_drive_stiffness.is_changed() ||
				joint_component.slerp_drive_force_limit.is_changed())
				&& joint_component.perform_slerp_drive)
			{
				PxD6JointDrive drive;
				drive.stiffness = joint_component.slerp_drive_stiffness;
				drive.damping = joint_component.slerp_drive_damping;
				drive.forceLimit = joint_component.slerp_drive_force_limit;
				drive.flags = static_cast<PxD6JointDriveFlags>(joint_component.slerp_drive_accelerated ? 1 : 0);
				native_joint->setDrive(PxD6Drive::eSLERP, drive);

				PxRigidActor* actor0 = nullptr;
				PxRigidActor* actor1 = nullptr;
				native_joint->getActors(actor0, actor1);

				if (PxRigidDynamic* actor_dynamic = actor0 == nullptr ? nullptr : actor0->is<PxRigidDynamic>(); actor_dynamic != nullptr)
				{
					if (!actor_dynamic->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION) &&
						!actor_dynamic->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
					{
						actor_dynamic->wakeUp();
					}
				}

				if (PxRigidDynamic* actor_dynamic = actor1 == nullptr ? nullptr : actor1->is<PxRigidDynamic>(); actor_dynamic != nullptr)
				{
					if (!actor_dynamic->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION) &&
						!actor_dynamic->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
					{
						actor_dynamic->wakeUp();
					}
				}

				joint_component.slerp_drive_damping.sync_changes();
				joint_component.slerp_drive_accelerated.sync_changes();
				joint_component.slerp_drive_stiffness.sync_changes();
				joint_component.slerp_drive_force_limit.sync_changes();
			}

			if (joint_component.linear_drive_accelerated.is_changed() ||
				joint_component.linear_drive_damping.is_changed() ||
				joint_component.linear_drive_stiffness.is_changed() ||
				joint_component.linear_drive_force_limit.is_changed())
			{
				PxD6JointDrive drive;
				drive.stiffness = joint_component.linear_drive_stiffness;
				drive.damping = joint_component.linear_drive_damping;
				drive.forceLimit = joint_component.linear_drive_force_limit;
				drive.flags = static_cast<PxD6JointDriveFlags>(joint_component.linear_drive_accelerated ? 1 : 0);
				native_joint->setDrive(PxD6Drive::eX, drive);
				native_joint->setDrive(PxD6Drive::eY, drive);
				native_joint->setDrive(PxD6Drive::eZ, drive);

				PxRigidActor* actor0 = nullptr;
				PxRigidActor* actor1 = nullptr;
				native_joint->getActors(actor0, actor1);

				if (PxRigidDynamic* actor_dynamic = actor0 == nullptr ? nullptr : actor0->is<PxRigidDynamic>(); actor_dynamic != nullptr)
				{
					if (!actor_dynamic->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION) &&
						!actor_dynamic->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
					{
						actor_dynamic->wakeUp();
					}
				}

				if (PxRigidDynamic* actor_dynamic = actor1 == nullptr ? nullptr : actor1->is<PxRigidDynamic>(); actor_dynamic != nullptr)
				{
					if (!actor_dynamic->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION) &&
						!actor_dynamic->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
					{
						actor_dynamic->wakeUp();
					}
				}

				joint_component.linear_drive_damping.sync_changes();
				joint_component.linear_drive_accelerated.sync_changes();
				joint_component.linear_drive_stiffness.sync_changes();
				joint_component.linear_drive_force_limit.sync_changes();
			}

			if (joint_component.angular_drive_velocity.is_changed() || 
				joint_component.linear_drive_velocity.is_changed())
			{
				native_joint->setDriveVelocity(create_PxVec3(joint_component.linear_drive_velocity), create_PxVec3(joint_component.angular_drive_velocity), false);

				PxRigidActor* actor0 = nullptr;
				PxRigidActor* actor1 = nullptr;
				native_joint->getActors(actor0, actor1);

				if (PxRigidDynamic* actor_dynamic = actor0 == nullptr ? nullptr : actor0->is<PxRigidDynamic>(); actor_dynamic != nullptr)
				{
					if (!actor_dynamic->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION) &&
						!actor_dynamic->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
					{
						actor_dynamic->wakeUp();
					}
				}

				if (PxRigidDynamic* actor_dynamic = actor1 == nullptr ? nullptr : actor1->is<PxRigidDynamic>(); actor_dynamic != nullptr)
				{
					if (!actor_dynamic->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION) &&
						!actor_dynamic->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
					{
						actor_dynamic->wakeUp();
					}
				}

				joint_component.angular_drive_velocity.sync_changes();
				joint_component.linear_drive_velocity.sync_changes();
			}

			if (joint_component.drive_transform.is_changed())
			{
				native_joint->setDrivePosition(create_PxTransform(joint_component.drive_transform), false);

				PxRigidActor* actor0 = nullptr;
				PxRigidActor* actor1 = nullptr;
				native_joint->getActors(actor0, actor1);

				if (PxRigidDynamic* actor_dynamic = actor0 == nullptr ? nullptr : actor0->is<PxRigidDynamic>(); actor_dynamic != nullptr)
				{
					if (!actor_dynamic->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION) &&
						!actor_dynamic->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
					{
						actor_dynamic->wakeUp();
					}
				}

				if (PxRigidDynamic* actor_dynamic = actor1 == nullptr ? nullptr : actor1->is<PxRigidDynamic>(); actor_dynamic != nullptr)
				{
					if (!actor_dynamic->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION) &&
						!actor_dynamic->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
					{
						actor_dynamic->wakeUp();
					}
				}

				joint_component.drive_transform.sync_changes();
			}
		}
	}

	void JointsSystem::process_added_joints()
	{
		using namespace physx;

		ScopedSpinLock _lock{ sync };

		for (auto iter = d6_joints_to_init.begin(); iter != d6_joints_to_init.end();)
		{
			Entity entity = world->get_entity(*iter);

			D6JointComponent* joint_component = entity.get_component<D6JointComponent>();

			PxRigidActor* first_actor = nullptr;
			PxRigidActor* second_actor = nullptr;

			if (!joint_component->base_descriptor.connected_entity.is_empty() &&
				!joint_component->base_descriptor.second_connected_entity.is_empty())
			{
				first_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.connected_entity.get())->get_rigid_actor();
				second_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.second_connected_entity.get())->get_rigid_actor();

				if (first_actor == nullptr ||
					second_actor == nullptr)
				{
					++iter;
					continue;
				}
			}
			else
			{
				if (!joint_component->base_descriptor.connected_entity.is_empty())
				{
					first_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.connected_entity.get())->get_rigid_actor();
					if (first_actor == nullptr)
					{
						++iter;
						continue;
					}
				}
				else if (!joint_component->base_descriptor.second_connected_entity.is_empty())
				{
					second_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.second_connected_entity.get())->get_rigid_actor();
					if (second_actor == nullptr)
					{
						++iter;
						continue;
					}

				}
				else
				{
					ASSERT(false);
					iter = d6_joints_to_init.erase(iter);
					continue;
				}
			}

			PxD6Joint* created_joint = PxD6JointCreate(*PhysicsHolder::physics_ref->get_physics(),
				first_actor,
				create_PxTransform(joint_component->base_descriptor.local_frame),
				second_actor,
				create_PxTransform(joint_component->base_descriptor.second_local_frame));

			created_joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, false);
			created_joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, joint_component->enable_collision);

			joint_component->joint = created_joint;
			joint_component->joint->userData = joint_component;
						
			joint_component->state = JointComponent::State::ENABLED;

			iter = d6_joints_to_init.erase(iter);
		}

		for (auto iter = distance_joints_to_init.begin(); iter != distance_joints_to_init.end();)
		{
			Entity entity = world->get_entity(*iter);

			DistanceJointComponent* joint_component = entity.get_component<DistanceJointComponent>();

			PxRigidActor* first_actor = nullptr;
			PxRigidActor* second_actor = nullptr;

			if (!joint_component->base_descriptor.connected_entity.is_empty() &&
				!joint_component->base_descriptor.second_connected_entity.is_empty())
			{
				first_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.connected_entity.get())->get_rigid_actor();
				second_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.second_connected_entity.get())->get_rigid_actor();

				if (first_actor == nullptr ||
					second_actor == nullptr)
				{
					++iter;
					continue;
				}
			}
			else
			{
				if (!joint_component->base_descriptor.connected_entity.is_empty())
				{
					first_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.connected_entity.get())->get_rigid_actor();
					if (first_actor == nullptr)
					{
						++iter;
						continue;
					}
				}
				else if (!joint_component->base_descriptor.second_connected_entity.is_empty())
				{
					second_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.second_connected_entity.get())->get_rigid_actor();
					if (second_actor == nullptr)
					{
						++iter;
						continue;
					}

				}
				else
				{
					ASSERT(false);
					iter = d6_joints_to_init.erase(iter);
					continue;
				}
			}

			PxDistanceJoint* created_joint = PxDistanceJointCreate(*PhysicsHolder::physics_ref->get_physics(),
				first_actor,
				create_PxTransform(joint_component->base_descriptor.local_frame),
				second_actor,
				create_PxTransform(joint_component->base_descriptor.second_local_frame));

			created_joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, false);
			created_joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, joint_component->enable_collision);

			joint_component->joint = created_joint;
			joint_component->joint->userData = joint_component;

			joint_component->state = JointComponent::State::ENABLED;

			iter = distance_joints_to_init.erase(iter);
		}

		for (auto iter = fixed_joints_to_init.begin(); iter != fixed_joints_to_init.end();)
		{
			Entity entity = world->get_entity(*iter);

			FixedJointComponent* joint_component = entity.get_component<FixedJointComponent>();

			PxRigidActor* first_actor = nullptr;
			PxRigidActor* second_actor = nullptr;

			if (!joint_component->base_descriptor.connected_entity.is_empty() &&
				!joint_component->base_descriptor.second_connected_entity.is_empty())
			{
				first_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.connected_entity.get())->get_rigid_actor();
				second_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.second_connected_entity.get())->get_rigid_actor();

				if (first_actor == nullptr ||
					second_actor == nullptr)
				{
					++iter;
					continue;
				}
			}
			else
			{
				if (!joint_component->base_descriptor.connected_entity.is_empty())
				{
					first_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.connected_entity.get())->get_rigid_actor();
					if (first_actor == nullptr)
					{
						++iter;
						continue;
					}
				}
				else if (!joint_component->base_descriptor.second_connected_entity.is_empty())
				{
					second_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.second_connected_entity.get())->get_rigid_actor();
					if (second_actor == nullptr)
					{
						++iter;
						continue;
					}

				}
				else
				{
					ASSERT(false);
					iter = d6_joints_to_init.erase(iter);
					continue;
				}
			}

			PxFixedJoint* created_joint = PxFixedJointCreate(*PhysicsHolder::physics_ref->get_physics(),
				first_actor,
				create_PxTransform(joint_component->base_descriptor.local_frame),
				second_actor,
				create_PxTransform(joint_component->base_descriptor.second_local_frame));

			created_joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, false);
			created_joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, joint_component->enable_collision);

			joint_component->joint = created_joint;
			joint_component->joint->userData = joint_component;

			joint_component->state = JointComponent::State::ENABLED;

			iter = fixed_joints_to_init.erase(iter);
		}

		for (auto iter = revolute_joints_to_init.begin(); iter != revolute_joints_to_init.end();)
		{
			Entity entity = world->get_entity(*iter);

			RevoluteJointComponent* joint_component = entity.get_component<RevoluteJointComponent>();

			PxRigidActor* first_actor = nullptr;
			PxRigidActor* second_actor = nullptr;

			if (!joint_component->base_descriptor.connected_entity.is_empty() &&
				!joint_component->base_descriptor.second_connected_entity.is_empty())
			{
				first_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.connected_entity.get())->get_rigid_actor();
				second_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.second_connected_entity.get())->get_rigid_actor();

				if (first_actor == nullptr ||
					second_actor == nullptr)
				{
					++iter;
					continue;
				}
			}
			else
			{
				if (!joint_component->base_descriptor.connected_entity.is_empty())
				{
					first_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.connected_entity.get())->get_rigid_actor();
					if (first_actor == nullptr)
					{
						++iter;
						continue;
					}
				}
				else if (!joint_component->base_descriptor.second_connected_entity.is_empty())
				{
					second_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.second_connected_entity.get())->get_rigid_actor();
					if (second_actor == nullptr)
					{
						++iter;
						continue;
					}

				}
				else
				{
					ASSERT(false);
					iter = d6_joints_to_init.erase(iter);
					continue;
				}
			}

			PxRevoluteJoint* created_joint = PxRevoluteJointCreate(*PhysicsHolder::physics_ref->get_physics(),
				first_actor,
				create_PxTransform(joint_component->base_descriptor.local_frame),
				second_actor,
				create_PxTransform(joint_component->base_descriptor.second_local_frame));

			created_joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, false);
			created_joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, joint_component->enable_collision);

			joint_component->joint = created_joint;
			joint_component->joint->userData = joint_component;

			joint_component->state = JointComponent::State::ENABLED;

			iter = revolute_joints_to_init.erase(iter);
		}

		for (auto iter = spherical_joints_to_init.begin(); iter != spherical_joints_to_init.end();)
		{
			Entity entity = world->get_entity(*iter);

			SphericalJointComponent* joint_component = entity.get_component<SphericalJointComponent>();

			PxRigidActor* first_actor = nullptr;
			PxRigidActor* second_actor = nullptr;

			if (!joint_component->base_descriptor.connected_entity.is_empty() &&
				!joint_component->base_descriptor.second_connected_entity.is_empty())
			{
				first_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.connected_entity.get())->get_rigid_actor();
				second_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.second_connected_entity.get())->get_rigid_actor();

				if (first_actor == nullptr ||
					second_actor == nullptr)
				{
					++iter;
					continue;
				}
			}
			else
			{
				if (!joint_component->base_descriptor.connected_entity.is_empty())
				{
					first_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.connected_entity.get())->get_rigid_actor();
					if (first_actor == nullptr)
					{
						++iter;
						continue;
					}
				}
				else if (!joint_component->base_descriptor.second_connected_entity.is_empty())
				{
					second_actor = PhysicsUtils::get_body_component(joint_component->base_descriptor.second_connected_entity.get())->get_rigid_actor();
					if (second_actor == nullptr)
					{
						++iter;
						continue;
					}

				}
				else
				{
					ASSERT(false);
					iter = d6_joints_to_init.erase(iter);
					continue;
				}
			}

			PxSphericalJoint* created_joint = PxSphericalJointCreate(*PhysicsHolder::physics_ref->get_physics(),
				first_actor,
				create_PxTransform(joint_component->base_descriptor.local_frame),
				second_actor,
				create_PxTransform(joint_component->base_descriptor.second_local_frame));

			created_joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, false);
			created_joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, joint_component->enable_collision);

			joint_component->joint = created_joint;
			joint_component->joint->userData = joint_component;

			joint_component->state = JointComponent::State::ENABLED;

			iter = spherical_joints_to_init.erase(iter);
		}
	}

	void JointsSystem::on_fixed_joint_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		fixed_joints_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}

	void JointsSystem::on_distance_joint_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		distance_joints_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}

	void JointsSystem::on_d6_joint_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		d6_joints_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}

	void JointsSystem::on_spherical_joint_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		spherical_joints_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}

	void JointsSystem::on_revolute_joint_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		revolute_joints_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}

}
