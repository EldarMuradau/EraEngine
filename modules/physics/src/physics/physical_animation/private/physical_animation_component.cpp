#include "physics/physical_animation/physical_animation_component.h"
#include "physics/body_component.h"
#include "physics/joint.h"
#include "physics/physical_animation/states/blend_in_simulation_state.h"
#include "physics/physical_animation/states/blend_out_simulation_state.h"
#include "physics/physical_animation/states/enabled_simulation_state.h"
#include "physics/physical_animation/states/disabled_simulation_state.h"
#include "physics/physical_animation/states/ragdoll_simulation_state.h"
#include "physics/physical_animation/limb_states/blend_out_limb_state.h"
#include "physics/physical_animation/limb_states/kinematic_limb_state.h"
#include "physics/physical_animation/limb_states/simulation_limb_state.h"
#include "physics/physical_animation/limb_states/ragdoll_limb_state.h"

#include <rttr/registration>

namespace era_engine::physics
{

	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<PhysicalAnimationComponent>("PhysicalAnimationComponent")
			.constructor<>();

		registration::class_<PhysicalAnimationLimbComponent>("PhysicalAnimationLimbComponent")
			.constructor<>();
	}

	bool PhysicsLimbChain::has_any_colliding_limb() const
	{
		for (EntityPtr limb_ptr : connected_limbs)
		{
			Entity limb = limb_ptr.get();

			const PhysicalAnimationLimbComponent* limb_component = limb.get_component<PhysicalAnimationLimbComponent>();
			ASSERT(limb_component != nullptr);

			if (limb_component->is_colliding || limb_component->collision_time > 0.0f)
			{
				return true;
			}
		}
		return false;
	}

	float PhysicalAnimationLimbComponent::calculate_desired_angular_damping(float delta_angle) const
	{
		const float angular_damping_value = map_value(delta_angle,
			angular_range.x,
			angular_range.y,
			angular_damping_range.x,
			angular_damping_range.y);

		return angular_damping_value * RagdollStrengthConfig::ANGULAR_DAMPING_MODIFIER;
	}

	float PhysicalAnimationLimbComponent::calculate_desired_linear_damping(float delta_position) const
	{
		const float linear_damping_value = map_value(delta_position,
			linear_range.x,
			linear_range.y,
			linear_damping_range.x,
			linear_damping_range.y);

		return linear_damping_value * RagdollStrengthConfig::LINEAR_DAMPING_MODIFIER;
	}

	void PhysicalAnimationLimbComponent::reset_collision_data()
	{
		was_in_collision = false;
		is_colliding = false;
		collision_time = 0.0f;
	}

	PhysicalAnimationLimbComponent::PhysicalAnimationLimbComponent(ref<Entity::EcsData> _data, uint32 _joint_id /*= INVALID_JOINT*/)
		: RagdollLimbComponent(_data, _joint_id)
	{
		ComponentPtr this_component_ptr = ComponentPtr(this);

		simulation_states.emplace(PhysicalLimbStateType::KINEMATIC, std::make_shared<KinematicLimbState>(this_component_ptr));
		simulation_states.emplace(PhysicalLimbStateType::BLEND_OUT, std::make_shared<BlendOutLimbState>(this_component_ptr));
		simulation_states.emplace(PhysicalLimbStateType::SIMULATION, std::make_shared<SimulationLimbState>(this_component_ptr));
		simulation_states.emplace(PhysicalLimbStateType::RAGDOLL, std::make_shared<RagdollLimbState>(this_component_ptr));
	}

	PhysicalAnimationLimbComponent::~PhysicalAnimationLimbComponent()
	{
	}

	std::shared_ptr<BaseLimbState> PhysicalAnimationLimbComponent::get_current_state() const
	{
		return simulation_states.at(current_state_type);
	}

	PhysicalLimbStateType PhysicalAnimationLimbComponent::get_current_state_type() const
	{
		return current_state_type;
	}

	void PhysicalAnimationLimbComponent::force_switch_state(PhysicalLimbStateType desired_state)
	{
		if(current_state_type != desired_state)
		{
			get_current_state()->on_exit();
			current_state_type = desired_state;
			get_current_state()->on_enter();
		}
	}

	void PhysicalAnimationLimbComponent::update_states(float dt, PhysicalLimbStateType desired_state)
	{
		if (current_state_type != desired_state)
		{
			const PhysicalLimbStateType transition_state = get_current_state()->try_switch_to(desired_state);

			force_switch_state(transition_state);
		}

		get_current_state()->update(dt);
	}

	PhysicalAnimationComponent::PhysicalAnimationComponent(ref<Entity::EcsData> _data)
		: RagdollComponent(_data)
	{
		ComponentPtr this_component_ptr = ComponentPtr(this);

		simulation_states.emplace(SimulationStateType::ENABLED, std::make_shared<EnabledSimulationState>(this_component_ptr));
		simulation_states.emplace(SimulationStateType::DISABLED, std::make_shared<DisabledSimulationState>(this_component_ptr));
		simulation_states.emplace(SimulationStateType::BLEND_IN, std::make_shared<BlendInSimulationState>(this_component_ptr));
		simulation_states.emplace(SimulationStateType::BLEND_OUT, std::make_shared<BlendOutSimulationState>(this_component_ptr));
		simulation_states.emplace(SimulationStateType::RAGDOLL, std::make_shared<RagdollSimulationState>(this_component_ptr));

		current_state_type = SimulationStateType::DISABLED;
	}

	PhysicalAnimationComponent::~PhysicalAnimationComponent()
	{
	}

	bool PhysicalAnimationComponent::try_to_apply_ragdoll_profile(ref<RagdollProfile> new_profile, RagdollProfileStrengthType new_strength_type/* = RagdollProfileStrengthType::DEFAULT*/, bool force_reload)
	{
		if (current_profile != new_profile ||
			strength_type != new_strength_type || 
			force_reload)
		{
			current_profile = new_profile;
			strength_type = new_strength_type;

			for (const EntityPtr& limb_ptr : limbs)
			{
				Entity limb = limb_ptr.get();
				PhysicalAnimationLimbComponent* limb_component = limb.get_component<PhysicalAnimationLimbComponent>();

				const PhysicalLimbDetails& limb_details = current_profile->get_limb_details_by_type(limb_component->type);

				limb_component->blend_type = limb_details.blend_type;

				if (limb_details.motor_drive.has_value())
				{
					const MotorDriveDetails& motor_drive = limb_details.motor_drive.value();

					limb_component->angular_range = motor_drive.angular_range;
					limb_component->linear_range = motor_drive.linear_range;

					float strength_coeff = limb_details.strength_details.default_strength_coeff;
					if (strength_type == RagdollProfileStrengthType::SOFT)
					{
						strength_coeff = limb_details.strength_details.soft_strength_coeff;
					}
					else if(strength_type == RagdollProfileStrengthType::HARD)
					{
						strength_coeff = limb_details.strength_details.hard_strength_coeff;
					}

					D6JointComponent* joint_component = dynamic_cast<D6JointComponent*>(limb_component->drive_joint_component.get_for_write());
					if (joint_component != nullptr)
					{
						if (motor_drive.enable_slerp_drive)
						{
							joint_component->slerp_drive_force_limit = motor_drive.max_force;
							joint_component->slerp_drive_stiffness = motor_drive.angular_drive_stiffness * strength_coeff * RagdollStrengthConfig::ANGULAR_STIFFNESS_MODIFIER;
							joint_component->slerp_drive_accelerated = motor_drive.accelerated;
						}
						else
						{
							joint_component->swing_drive_force_limit = motor_drive.max_force;
							joint_component->swing_drive_stiffness = motor_drive.angular_drive_stiffness * strength_coeff * RagdollStrengthConfig::ANGULAR_STIFFNESS_MODIFIER;
							joint_component->swing_drive_accelerated = motor_drive.accelerated;

							joint_component->twist_drive_force_limit = motor_drive.max_force;
							joint_component->twist_drive_stiffness = motor_drive.angular_drive_stiffness * strength_coeff * RagdollStrengthConfig::ANGULAR_STIFFNESS_MODIFIER;
							joint_component->twist_drive_accelerated = motor_drive.accelerated;
						}

						joint_component->linear_drive_force_limit = motor_drive.max_force;
						joint_component->linear_drive_stiffness = motor_drive.linear_drive_stiffness * strength_coeff * RagdollStrengthConfig::LINEAR_STIFFNESS_MODIFIER;
						joint_component->linear_drive_accelerated = motor_drive.accelerated;
					}
				}

			}

			// If we update ragdoll profile, we need to combine prev and current blend types untill ragdoll_profile_transition_time is over.
			ragdoll_profile_transition_time = MAX_RAGDOLL_PROFILE_TRANSITION_TIME;

			return true;
		}
		return false;
	}

	float PhysicalAnimationComponent::get_profile_transition_time() const
	{
		return ragdoll_profile_transition_time;
	}

	bool PhysicalAnimationComponent::is_in_transition() const
	{
		return ragdoll_profile_transition_time > 0.0f;
	}

	void PhysicalAnimationComponent::update_profile_transition(float dt)
	{
		if (fuzzy_equals(ragdoll_profile_transition_time, 0.0f))
		{
			return;
		}

		if (ragdoll_profile_transition_time > 0.0f)
		{
			ragdoll_profile_transition_time -= dt;
		}

		if (ragdoll_profile_transition_time < 0.0f)
		{
			ragdoll_profile_transition_time = 0.0f;

			for (auto& limb_ptr : limbs)
			{
				PhysicalAnimationLimbComponent* limb_component = limb_ptr.get().get_component<PhysicalAnimationLimbComponent>();
				ASSERT(limb_component != nullptr);

				limb_component->prev_blend_type = PhysicalLimbBlendType::NONE;
			}
		}
	}

	ref<RagdollProfile> PhysicalAnimationComponent::get_ragdoll_profile() const
	{
		return current_profile;
	}

	std::shared_ptr<BaseSimulationState> PhysicalAnimationComponent::get_current_state() const
	{
		return simulation_states.at(current_state_type);
	}

	SimulationStateType PhysicalAnimationComponent::get_current_state_type() const
	{
		return current_state_type;
	}

	void PhysicalAnimationComponent::force_switch_state(SimulationStateType desired_state)
	{
		if(current_state_type != desired_state)
		{
			get_current_state()->on_exit();
			current_state_type = desired_state;
			get_current_state()->on_enter();
		}
	}

	bool PhysicalAnimationComponent::is_in_idle() const
	{
		return current_profile->type == RagdollProfileType::IDLE;
	}

	bool PhysicalAnimationComponent::is_in_ragdoll() const
	{
		return force_ragdoll_state;
	}

	void PhysicalAnimationComponent::force_set_ragdoll(bool enabled)
	{
		force_ragdoll_state = enabled;
	}

	void PhysicalAnimationComponent::update_states(float dt, SimulationStateType desired_state)
	{
		if (current_state_type != desired_state)
		{
			const SimulationStateType transition_state = get_current_state()->try_switch_to(desired_state);

			force_switch_state(transition_state);
		}

		get_current_state()->update(dt);

		update_profile_transition(dt);
	}

}