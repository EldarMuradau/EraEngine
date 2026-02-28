#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"

#include <ecs/component.h>
#include <ecs/observable_member.h>

namespace era_engine::physics
{
	physx::PxRevoluteJoint* create_revolute_joint(physx::PxRigidActor* first_actor, physx::PxRigidActor* second_actor);
	physx::PxDistanceJoint* create_distance_joint(physx::PxRigidActor* first_actor, physx::PxRigidActor* second_actor);
	physx::PxSphericalJoint* create_spherical_joint(physx::PxRigidActor* first_actor, physx::PxRigidActor* second_actor);
	physx::PxFixedJoint* create_fixed_joint(physx::PxRigidActor* first_actor, physx::PxRigidActor* second_actor);
	physx::PxPrismaticJoint* create_prismatic_joint(physx::PxRigidActor* first_actor, physx::PxRigidActor* second_actor);
	physx::PxD6Joint* create_d6_joint(physx::PxRigidActor* first_actor, physx::PxRigidActor* second_actor);

	class ERA_PHYSICS_API JointComponent : public Component
	{
	public:
		enum class State : uint8_t
		{
			ENABLED,
			DISABLED,
			BROKEN
		};

		struct ERA_PHYSICS_API BaseDescriptor
		{
			EntityPtr connected_entity;
			EntityPtr second_connected_entity;

			trs local_frame = trs::identity;
			trs second_local_frame = trs::identity;
		};

		JointComponent() = default;
		JointComponent(ref<Entity::EcsData> _data, const BaseDescriptor& _base_descriptor);

		~JointComponent() override;

		State get_state() const;

		physx::PxJoint* get_native_joint() const;

		EntityPtr get_first_entity_ptr() const;
		EntityPtr get_second_entity_ptr() const;

		const trs& get_first_local_frame() const;
		const trs& get_second_local_frame() const;

		ObservableMember<bool> enable_collision = false;

		ObservableMember<bool> always_update = false;

		ObservableMember<float> break_force = std::numeric_limits<float>::max();

		ObservableMember<bool> disabled = false;
		ObservableMember<bool> disable_preprocessing = false;

		std::function<void(JointComponent*)> on_broken_callback = nullptr;

		ERA_VIRTUAL_REFLECT(Component)

	protected:
		physx::PxJoint* joint = nullptr;
		BaseDescriptor base_descriptor;
		State state = State::DISABLED;

		friend class JointsSystem;
		friend class SimulationEventCallback;
	};

	class ERA_PHYSICS_API FixedJointComponent : public JointComponent
	{
	public:
		FixedJointComponent() = default;
		FixedJointComponent(ref<Entity::EcsData> _data, const JointComponent::BaseDescriptor& _base_descriptor);
		~FixedJointComponent() override;

		ERA_VIRTUAL_REFLECT(JointComponent)
	};

	class ERA_PHYSICS_API RevoluteJointComponent : public JointComponent
	{
	public:
		RevoluteJointComponent() = default;
		RevoluteJointComponent(ref<Entity::EcsData> _data, const JointComponent::BaseDescriptor& _base_descriptor);
		~RevoluteJointComponent() override;

		ObservableMember<bool> enable_drive = false;

		ObservableMember<float> drive_velocity = 0.0f;
		ObservableMember<float> drive_force_limit = 1000.0f;
		ObservableMember<float> drive_gear_ratio = 0.0f;

		//x = lower, y = upper
		ObservableMember<vec2> linear_limit = vec2::zero;

		ERA_VIRTUAL_REFLECT(JointComponent)
	};

	class ERA_PHYSICS_API DistanceJointComponent : public JointComponent
	{
	public:
		DistanceJointComponent() = default;
		DistanceJointComponent(ref<Entity::EcsData> _data, const JointComponent::BaseDescriptor& _base_descriptor);
		~DistanceJointComponent() override;

		ObservableMember<float> stiffness = 0.0f;
		ObservableMember<float> damping = 0.0f;

		ObservableMember<float> min_distance = 0.0f;
		ObservableMember<float> max_distance = 0.0f;

		ObservableMember<bool> spring_enabled = false;

		ERA_VIRTUAL_REFLECT(JointComponent)
	};

	class ERA_PHYSICS_API SphericalJointComponent : public JointComponent
	{
	public:
		SphericalJointComponent() = default;
		SphericalJointComponent(ref<Entity::EcsData> _data, const JointComponent::BaseDescriptor& _base_descriptor);
		~SphericalJointComponent() override;

		//x = swing_y, y = swing_z
		ObservableMember<vec2> angular_limit = vec2::zero;

		ERA_VIRTUAL_REFLECT(JointComponent)
	};

	class ERA_PHYSICS_API D6JointComponent : public JointComponent
	{
	public:
		enum class Motion : uint8
		{
			LOCKED = 0,	// The DOF is locked, it does not allow relative motion.
			LIMITED,	// The DOF is limited, it only allows motion within a specific range.
			FREE		// The DOF is free and has its full range of motion.
		};

		D6JointComponent() = default;
		D6JointComponent(ref<Entity::EcsData> _data, const JointComponent::BaseDescriptor& _base_descriptor);
		~D6JointComponent() override;

		bool perform_slerp_drive = false;

		ObservableMember<float> swing_y_limit = 0.0f;
		ObservableMember<float> swing_z_limit = 0.0f;

		ObservableMember<float> twist_min_limit = 0.0f;
		ObservableMember<float> twist_max_limit = 0.0f;

		ObservableMember<float> linear_limit = 0.00f;
		ObservableMember<float> distance_limit = 0.00f;

		ObservableMember<Motion> linear_x_motion_type = Motion::FREE;
		ObservableMember<Motion> linear_y_motion_type = Motion::FREE;
		ObservableMember<Motion> linear_z_motion_type = Motion::FREE;

		ObservableMember<float> swing_limit_stiffness = 0.0f;
		ObservableMember<float> swing_limit_damping = 0.0f;
		ObservableMember<float> swing_limit_restitution = 0.0f;

		ObservableMember<float> twist_limit_stiffness = 0.0f;
		ObservableMember<float> twist_limit_damping = 0.0f;
		ObservableMember<float> twist_limit_restitution = 0.0f;

		ObservableMember<Motion> swing_y_motion_type = Motion::FREE;
		ObservableMember<Motion> swing_z_motion_type = Motion::FREE;
		ObservableMember<Motion> twist_motion_type = Motion::FREE;

		ObservableMember<float> slerp_drive_force_limit = 0.0f;
		ObservableMember<float> swing_drive_force_limit = 0.0f;
		ObservableMember<float> twist_drive_force_limit = 0.0f;
		ObservableMember<float> linear_drive_force_limit = 0.0f;

		ObservableMember<bool> drive_limits_are_forces = false;

		ObservableMember<bool> improved_slerp = false;

		ObservableMember<float> slerp_drive_stiffness = 0.0f;
		ObservableMember<float> swing_drive_stiffness = 0.0f;
		ObservableMember<float> twist_drive_stiffness = 0.0f;
		ObservableMember<float> linear_drive_stiffness = 0.0f;

		ObservableMember<float> slerp_drive_damping = 0.0f;
		ObservableMember<float> swing_drive_damping = 0.0f;
		ObservableMember<float> twist_drive_damping = 0.0f;
		ObservableMember<float> linear_drive_damping = 0.0f;

		ObservableMember<bool> slerp_drive_accelerated = false;
		ObservableMember<bool> swing_drive_accelerated = false;
		ObservableMember<bool> twist_drive_accelerated = false;
		ObservableMember<bool> linear_drive_accelerated = false;

		ObservableMember<vec3> angular_drive_velocity = vec3::zero;
		ObservableMember<vec3> linear_drive_velocity = vec3::zero;

		ObservableMember<trs> drive_transform = trs::identity;

		ERA_VIRTUAL_REFLECT(JointComponent)
	};
}