#include "physics/joint.h"
#include "physics/core/physics.h"
#include "physics/core/physics_utils.h"
#include "physics/body_component.h"

#include <rttr/registration>

namespace era_engine::physics
{

	physx::PxRevoluteJoint* create_revolute_joint(physx::PxRigidActor* first_actor, physx::PxRigidActor* second_actor)
	{
		auto* physics = PhysicsHolder::physics_ref->get_physics();

		return physx::PxRevoluteJointCreate(*physics,
			first_actor, first_actor->getGlobalPose(),
			second_actor, second_actor->getGlobalPose());
	}

	physx::PxDistanceJoint* create_distance_joint(physx::PxRigidActor* first_actor, physx::PxRigidActor* second_actor)
	{
		auto* physics = PhysicsHolder::physics_ref->get_physics();

		return physx::PxDistanceJointCreate(*physics,
			first_actor, first_actor->getGlobalPose(),
			second_actor, second_actor->getGlobalPose());
	}

	physx::PxSphericalJoint* create_spherical_joint(physx::PxRigidActor* first_actor, physx::PxRigidActor* second_actor)
	{
		auto* physics = PhysicsHolder::physics_ref->get_physics();

		return physx::PxSphericalJointCreate(*physics,
			first_actor, first_actor->getGlobalPose(),
			second_actor, second_actor->getGlobalPose());
	}

	physx::PxFixedJoint* create_fixed_joint(physx::PxRigidActor* first_actor, physx::PxRigidActor* second_actor)
	{
		auto* physics = PhysicsHolder::physics_ref->get_physics();

		return physx::PxFixedJointCreate(*physics,
			first_actor, first_actor->getGlobalPose(),
			second_actor, second_actor->getGlobalPose());
	}

	physx::PxPrismaticJoint* create_prismatic_joint(physx::PxRigidActor* first_actor, physx::PxRigidActor* second_actor)
	{
		auto* physics = PhysicsHolder::physics_ref->get_physics();

		return physx::PxPrismaticJointCreate(*physics,
			first_actor, first_actor->getGlobalPose(),
			second_actor, second_actor->getGlobalPose());
	}

	physx::PxD6Joint* create_d6_joint(physx::PxRigidActor* first_actor, physx::PxRigidActor* second_actor)
	{
		auto* physics = PhysicsHolder::physics_ref->get_physics();

		return physx::PxD6JointCreate(*physics,
			first_actor, first_actor->getGlobalPose(),
			second_actor, second_actor->getGlobalPose());
	}

	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::enumeration<JointComponent::State>("State")
		(
			value("ENABLED", JointComponent::State::ENABLED),
			value("DISABLED", JointComponent::State::DISABLED),
			value("BROKEN", JointComponent::State::BROKEN)
		);

		registration::class_<JointComponent>("JointComponent")
			.constructor<>();

		registration::class_<FixedJointComponent>("FixedJointComponent")
			.constructor<>();

		registration::class_<RevoluteJointComponent>("RevoluteJointComponent")
			.constructor<>();

		registration::class_<DistanceJointComponent>("DistanceJointComponent")
			.constructor<>();

		registration::class_<SphericalJointComponent>("SphericalJointComponent")
			.constructor<>();

		registration::class_<D6JointComponent>("D6JointComponent")
			.constructor<>();
	}

	JointComponent::JointComponent(ref<Entity::EcsData> _data, const BaseDescriptor& _base_descriptor)
		: Component(_data), base_descriptor(_base_descriptor)
	{
		enable_collision.set_component(ComponentPtr(this));
		break_force.set_component(ComponentPtr(this));
		always_update.set_component(ComponentPtr(this));
		disabled.set_component(ComponentPtr(this));
		disable_preprocessing.set_component(ComponentPtr(this));
	}

	JointComponent::~JointComponent()
	{
		PX_RELEASE(joint)
	}

	JointComponent::State JointComponent::get_state() const
	{
		return state;
	}

	physx::PxJoint* JointComponent::get_native_joint() const
	{
		return joint;
	}

	EntityPtr JointComponent::get_first_entity_ptr() const
	{
		return base_descriptor.connected_entity;
	}

	EntityPtr JointComponent::get_second_entity_ptr() const
	{
		return base_descriptor.second_connected_entity;
	}

	const trs& JointComponent::get_first_local_frame() const
	{
		return base_descriptor.local_frame;
	}

	const trs& JointComponent::get_second_local_frame() const
	{
		return base_descriptor.second_local_frame;
	}

	FixedJointComponent::FixedJointComponent(ref<Entity::EcsData> _data, const JointComponent::BaseDescriptor& _base_descriptor)
		: JointComponent(_data, _base_descriptor)
	{
	}

	FixedJointComponent::~FixedJointComponent()
	{
	}

	RevoluteJointComponent::RevoluteJointComponent(ref<Entity::EcsData> _data, const JointComponent::BaseDescriptor& _base_descriptor)
		: JointComponent(_data, _base_descriptor)
	{
		drive_force_limit.set_component(ComponentPtr(this));
		drive_gear_ratio.set_component(ComponentPtr(this));
		drive_velocity.set_component(ComponentPtr(this));
		linear_limit.set_component(ComponentPtr(this));
	}

	RevoluteJointComponent::~RevoluteJointComponent()
	{
	}

	DistanceJointComponent::DistanceJointComponent(ref<Entity::EcsData> _data, const JointComponent::BaseDescriptor& _base_descriptor)
		: JointComponent(_data, _base_descriptor)
	{
		stiffness.set_component(ComponentPtr(this));
		damping.set_component(ComponentPtr(this));
		spring_enabled.set_component(ComponentPtr(this));
		max_distance.set_component(ComponentPtr(this));
		min_distance.set_component(ComponentPtr(this));
	}

	DistanceJointComponent::~DistanceJointComponent()
	{
	}
	
	SphericalJointComponent::SphericalJointComponent(ref<Entity::EcsData> _data, const JointComponent::BaseDescriptor& _base_descriptor)
		: JointComponent(_data, _base_descriptor)
	{
		angular_limit.set_component(ComponentPtr(this));
	}

	SphericalJointComponent::~SphericalJointComponent()
	{
	}

	D6JointComponent::D6JointComponent(ref<Entity::EcsData> _data, const JointComponent::BaseDescriptor& _base_descriptor)
		: JointComponent(_data, _base_descriptor)
	{
		drive_transform.set_component(ComponentPtr(this));
		linear_drive_velocity.set_component(ComponentPtr(this));
		angular_drive_velocity.set_component(ComponentPtr(this));

		linear_drive_accelerated.set_component(ComponentPtr(this));
		twist_drive_accelerated.set_component(ComponentPtr(this));
		swing_drive_accelerated.set_component(ComponentPtr(this));
		slerp_drive_accelerated.set_component(ComponentPtr(this));

		linear_drive_stiffness.set_component(ComponentPtr(this));
		twist_drive_stiffness.set_component(ComponentPtr(this));
		swing_drive_stiffness.set_component(ComponentPtr(this));
		slerp_drive_stiffness.set_component(ComponentPtr(this));

		linear_drive_force_limit.set_component(ComponentPtr(this));
		twist_drive_force_limit.set_component(ComponentPtr(this));
		swing_drive_force_limit.set_component(ComponentPtr(this));
		slerp_drive_force_limit.set_component(ComponentPtr(this));

		twist_motion_type.set_component(ComponentPtr(this));
		swing_z_motion_type.set_component(ComponentPtr(this));
		swing_y_motion_type.set_component(ComponentPtr(this));

		linear_x_motion_type.set_component(ComponentPtr(this));
		linear_y_motion_type.set_component(ComponentPtr(this));
		linear_z_motion_type.set_component(ComponentPtr(this));

		swing_z_limit.set_component(ComponentPtr(this));
		swing_y_limit.set_component(ComponentPtr(this));

		twist_max_limit.set_component(ComponentPtr(this));
		twist_min_limit.set_component(ComponentPtr(this));
		linear_z_motion_type.set_component(ComponentPtr(this));
		linear_z_motion_type.set_component(ComponentPtr(this));

		linear_limit.set_component(ComponentPtr(this));
		distance_limit.set_component(ComponentPtr(this));

		drive_limits_are_forces.set_component(ComponentPtr(this));

		improved_slerp.set_component(ComponentPtr(this));

		swing_limit_damping.set_component(ComponentPtr(this));
		swing_limit_stiffness.set_component(ComponentPtr(this));
		swing_limit_restitution.set_component(ComponentPtr(this));

		twist_limit_damping.set_component(ComponentPtr(this));
		twist_limit_stiffness.set_component(ComponentPtr(this));
		twist_limit_restitution.set_component(ComponentPtr(this));
	}

	D6JointComponent::~D6JointComponent()
	{
	}

}