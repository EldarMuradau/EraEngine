#pragma once

#include "physics_api.h"
#include "physics/physx_api.h"
#include "physics/ragdolls/ragdoll_component.h"
#include "physics/physical_animation/ragdoll_profile.h"
#include "physics/physical_animation/states/base_simulation_state.h"
#include "physics/physical_animation/limb_states/base_limb_state.h"

#include <core/math.h>

#include <ecs/component.h>
#include <ecs/observable_member.h>

namespace era_engine::physics
{
	class ERA_PHYSICS_API PhysicsLimbChain final
	{
	public:
		PhysicsLimbChain() = default;

		bool has_any_colliding_limb() const;

	public:
		std::vector<EntityPtr> connected_limbs;
	};

	class ERA_PHYSICS_API PhysicalAnimationLimbComponent : public RagdollLimbComponent
	{
	public:
		PhysicalAnimationLimbComponent() = default;
		PhysicalAnimationLimbComponent(ref<Entity::EcsData> _data, uint32 _joint_id = INVALID_JOINT);
		~PhysicalAnimationLimbComponent() override;

		std::shared_ptr<BaseLimbState> get_current_state() const;

		PhysicalLimbStateType get_current_state_type() const;

		void force_switch_state(PhysicalLimbStateType desired_state);

		void update_states(float dt, PhysicalLimbStateType desired_state);

		float calculate_desired_angular_damping(float delta_angle) const;
		float calculate_desired_linear_damping(float delta_position) const;

		void reset_collision_data();

	public:
		/* Joint local space transfrom from animation. */
		trs target_pose = trs::identity;

		vec2 angular_range = vec2(deg2rad(3.0f), deg2rad(30.0f));
		vec2 angular_damping_range = vec2(0.0f, 0.0f);

		vec2 linear_range = vec2(0.05f, 0.5f);
		vec2 linear_damping_range = vec2(0.0f, 0.0f);

		float transition_time = 1.0f;

		bool was_in_collision = false;
		bool is_colliding = false;
		float collision_time = 0.0f;

		constexpr static float MAX_COLLISION_TIME = 0.5f;
		constexpr static float MAX_FREQUENT_COLLISION_TIME = 0.3f;

		PhysicalLimbBlendType blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
		PhysicalLimbBlendType prev_blend_type = PhysicalLimbBlendType::NONE;

		ComponentPtr parent_joint_component = nullptr;
		ComponentPtr drive_joint_component = nullptr;

		EntityPtr ragdoll_ptr;
		EntityPtr drive_joint_entity_ptr;

        ERA_VIRTUAL_REFLECT(RagdollLimbComponent)

	private:
		PhysicalLimbStateType current_state_type = PhysicalLimbStateType::KINEMATIC;

		std::unordered_map<PhysicalLimbStateType, std::shared_ptr<BaseLimbState>> simulation_states;

		friend class PhysicalAnimationSystem;
	};

	class ERA_PHYSICS_API PhysicalAnimationComponent : public RagdollComponent
	{
	public:
		PhysicalAnimationComponent() = default;
		PhysicalAnimationComponent(ref<Entity::EcsData> _data);
		~PhysicalAnimationComponent() override;

		bool try_to_apply_ragdoll_profile(ref<RagdollProfile> new_profile, RagdollProfileStrengthType new_strength_type = RagdollProfileStrengthType::DEFAULT, bool force_reload = false);
		ref<RagdollProfile> get_ragdoll_profile() const;

		std::shared_ptr<BaseSimulationState> get_current_state() const;

		void reset_blends();

		SimulationStateType get_current_state_type() const;

		void force_switch_state(SimulationStateType desired_state);

		float get_profile_transition_time() const;

		bool is_in_transition() const;

		void update_profile_transition(float dt);

		bool is_in_idle() const;
		bool is_in_ragdoll() const;

		void force_set_ragdoll(bool enabled);

		void update_states(float dt, SimulationStateType desired_state);

	public:
		constexpr static float MAX_RAGDOLL_PROFILE_TRANSITION_TIME = 0.3f;

		bool use_fixed_pelvis_attachment = false;

		float blend_weight = 0.0f;

		float blend_in_time = 0.1f;
		float blend_out_time = 0.2f;

		trs prev_world_transform = trs::identity;

		vec3 velocity = vec3::zero;

		ref<PhysicsLimbChain> left_arm_chain;
		ref<PhysicsLimbChain> right_arm_chain;

		ref<PhysicsLimbChain> left_leg_chain;
		ref<PhysicsLimbChain> right_leg_chain;

		ref<PhysicsLimbChain> neck_chain;
		ref<PhysicsLimbChain> body_chain;

		EntityPtr attachment_body;

        ERA_VIRTUAL_REFLECT(RagdollComponent)

	private:
		ref<RagdollProfile> current_profile = nullptr;

		std::unordered_map<uint32, trs> local_joint_poses_for_target_calculation;

		SimulationStateType current_state_type = SimulationStateType::DISABLED;
		SkeletalMeshBlendType mesh_blend_type = SkeletalMeshBlendType::TRANSFORM;

		float ragdoll_profile_transition_time = 0.0f;

		std::unordered_map<SimulationStateType, std::shared_ptr<BaseSimulationState>> simulation_states;

		RagdollProfileStrengthType strength_type = RagdollProfileStrengthType::DEFAULT;
		bool force_ragdoll_state = false;

		friend class PhysicalAnimationSystem;
		friend class DrivePoseSampler;
		friend class PhysicalAnimationBuilderUtils;
	};
}