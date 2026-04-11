#include "physics/physical_animation/limb_states/simulation_limb_state.h"
#include "physics/physical_animation/physical_animation_component.h"
#include "physics/body_component.h"
#include "physics/joint.h"
#include "physics/shape_utils.h"
#include "physics/shape_component.h"
#include "physics/core/physics_utils.h"
#include "physics/physical_animation/physical_animation_utils.h"
#include "physics/physical_animation/ragdoll_profile.h"

#include <animation/animation.h>

#include <core/debug/debug_var.h>
#include <core/traits.h>

#include <ecs/base_components/transform_component.h>

namespace era_engine::physics
{
    SimulationLimbState::SimulationLimbState(ComponentPtr _limb_component_ptr)
        : BaseLimbState(_limb_component_ptr)
    {
    }

    void SimulationLimbState::update(float dt)
    {
        BaseLimbState::update(dt);

		PhysicalAnimationLimbComponent* limb_component = static_cast<PhysicalAnimationLimbComponent*>(physical_animation_limb_component_ptr.get_for_write());

		Entity limb = limb_component->get_entity();

        Entity ragdoll = limb_component->ragdoll_ptr.get();

        const PhysicalAnimationComponent* physical_animation_component = ragdoll.get_component<PhysicalAnimationComponent>();

        const ref<RagdollProfile> profile = physical_animation_component->get_ragdoll_profile();
        ASSERT(profile != nullptr);

		const PhysicalLimbDetails& limb_details = profile->get_limb_details_by_type(limb_component->type);

		// Motors drive for limb.
		if (limb_details.motor_drive.has_value())
		{
			ASSERT(limb_details.motor_drive->drive_type != MotorDriveType::NONE);

			ASSERT(!limb_component->drive_joint_component.is_empty());
			const MotorDriveDetails& motor_details = limb_details.motor_drive.value();
			D6JointComponent* drive_joint_component = static_cast<D6JointComponent*>(limb_component->drive_joint_component.get_for_write());

			const trs& constraint_frame_actor0_local_space = drive_joint_component->get_first_local_frame();
			const trs& constraint_frame_actor1_local_space = drive_joint_component->get_second_local_frame();

			const trs& parent_local_transform = drive_joint_component->get_first_entity_ptr().get().get_component<TransformComponent>()->get_local_transform();

			const trs constraint_frame_actor0_object_space = parent_local_transform * constraint_frame_actor0_local_space;
			const trs constraint_frame_actor1_object_space = limb_component->physics_pose * constraint_frame_actor1_local_space;

			const trs error_transform = invert(constraint_frame_actor1_object_space) * constraint_frame_actor0_object_space;
			const quat error_rotation = normalize(error_transform.rotation);
			const vec3 error_position = error_transform.position;

			float error_delta_angle = 0.0f;
			vec3 error_delta_axis = vec3::zero;
			get_axis_rotation(error_rotation, error_delta_axis, error_delta_angle);

			if (limb_details.motor_drive->drive_type == MotorDriveType::FULL)
			{
				drive_joint_component->drive_transform.get_for_write() = invert(constraint_frame_actor0_local_space) * constraint_frame_actor1_local_space;

				// Joint drive will handle drive velocity based on current limb velocity.
				drive_joint_component->angular_drive_velocity.get_for_write() = vec3::zero;
				drive_joint_component->linear_drive_velocity.get_for_write() = vec3::zero;
			}
			else
			{
				if (has_flag(limb_details.motor_drive->drive_type, MotorDriveType::TRANSFORM))
				{
					drive_joint_component->drive_transform.get_for_write() = invert(constraint_frame_actor0_local_space) * constraint_frame_actor1_local_space;
				}
				else if (has_flag(limb_details.motor_drive->drive_type, MotorDriveType::VELOCITY))
				{
					const vec3 angular_drive_velocity = normalize(error_delta_axis) * error_delta_angle / dt;
					const vec3 linear_drive_velocity = error_position / dt;

					drive_joint_component->angular_drive_velocity.get_for_write() = angular_drive_velocity;
					drive_joint_component->linear_drive_velocity.get_for_write() = linear_drive_velocity;
				}
			}

			const float angular_damping = limb_component->calculate_desired_angular_damping(error_delta_angle);

			if (drive_joint_component->perform_slerp_drive)
			{
				drive_joint_component->slerp_drive_damping = angular_damping;
			}
			else
			{
				drive_joint_component->twist_drive_damping = angular_damping;
				drive_joint_component->swing_drive_damping = angular_damping;
			}

			const float linear_damping = limb_component->calculate_desired_linear_damping(length(error_position));
			drive_joint_component->linear_drive_damping = linear_damping;
		}

		// Velocity drag frices for stabilization.
		if (limb_details.drag_force.has_value())
		{
			const DragForceDetails& drag_details = limb_details.drag_force.value();
			DynamicBodyComponent* dynamic_body = limb.get_component<DynamicBodyComponent>();

			const trs physics_target_delta_transform = invert(limb_component->physics_pose) * limb_component->target_pose;
			const quat physics_delta_rotation = normalize(physics_target_delta_transform.rotation);
			const vec3 physics_delta_position = physics_target_delta_transform.position;

			float physics_delta_angle = 0.0f;
			vec3 physics_delta_axis = vec3::zero;
			get_axis_rotation(physics_delta_rotation, physics_delta_axis, physics_delta_angle);

			const vec3 angular_drag_velocity = normalize(physics_delta_axis) * physics_delta_angle / dt;

			// Keyframe controller stage.
			{
				const vec3& raw_root_velocity = physical_animation_component->velocity;

				// Partial velocity drive.
				vec3 desired_velocity = physics_delta_position / dt;
				const float desired_velocity_magnitude = length(desired_velocity);
				if (desired_velocity_magnitude > drag_details.partial_velocity_drive_limit)
				{
					desired_velocity = desired_velocity * (drag_details.partial_velocity_drive_limit / desired_velocity_magnitude);
				}
				vec3 drive_linear_velocity = lerp(dynamic_body->linear_velocity, desired_velocity, drag_details.partial_velocity_drive);

				// Partial root velocity drive.
				vec3 root_velocity = raw_root_velocity;
				const float velocity_magnitude = length(root_velocity);
				if (velocity_magnitude > drag_details.acceleration_limit)
				{
					root_velocity = root_velocity * (drag_details.acceleration_limit / velocity_magnitude);
				}
				drive_linear_velocity += root_velocity * drag_details.acceleration_gain;

				dynamic_body->linear_velocity = drive_linear_velocity;
			}

			vec3 desired_angular_velocity = angular_drag_velocity;
			const float angular_velocity_magnitude = length(desired_angular_velocity);
			if (angular_velocity_magnitude > drag_details.partial_angular_drive_limit)
			{
				desired_angular_velocity *= (drag_details.partial_angular_drive_limit / angular_velocity_magnitude);
			}

			const float angular_velocity_modifier = drag_details.partial_angular_drive;

			const vec3& current_angular_velocity = dynamic_body->angular_velocity;
			dynamic_body->angular_velocity = lerp(current_angular_velocity, desired_angular_velocity, angular_velocity_modifier);
		}
    }

    PhysicalLimbStateType SimulationLimbState::try_switch_to(PhysicalLimbStateType desired_state) const
    {
        if (desired_state == PhysicalLimbStateType::KINEMATIC)
        {
            return PhysicalLimbStateType::BLEND_OUT;
        }
        return desired_state;
    }

	void SimulationLimbState::on_enter()
	{
		ASSERT(!physical_animation_limb_component_ptr.is_empty());

		PhysicalAnimationLimbComponent* limb_component = static_cast<PhysicalAnimationLimbComponent*>(physical_animation_limb_component_ptr.get_for_write());

		PhysicalAnimationUtils::reset_motor_drive(limb_component);
		PhysicalAnimationUtils::set_motor_drive_active(limb_component, true);

		BaseLimbState::on_enter();
	}
}