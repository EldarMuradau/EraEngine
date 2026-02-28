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

		const quat delta_rotation = normalize(limb_component->target_pose.rotation * conjugate(limb_component->physics_pose.rotation));
		const vec3 delta_position = limb_component->target_pose.position - limb_component->physics_pose.position;

		float delta_angle = 0.0f;
		vec3 delta_axis = vec3::zero;
		get_axis_rotation(delta_rotation, delta_axis, delta_angle);

		const vec3 angular_drive_velocity = normalize(delta_axis) * delta_angle / dt;

		const PhysicalLimbDetails& limb_details = profile->get_limb_details_by_type(limb_component->type);

		// Motors drive for limb.
		if (limb_details.motor_drive.has_value())
		{
			ASSERT(limb_details.motor_drive->drive_type != MotorDriveType::NONE);

			ASSERT(!limb_component->drive_joint_component.is_empty());
			const MotorDriveDetails& motor_details = limb_details.motor_drive.value();
			D6JointComponent* drive_joint_component = static_cast<D6JointComponent*>(limb_component->drive_joint_component.get_for_write());

			const trs& constraint_frame_in_actor0_local = drive_joint_component->get_first_local_frame();
			const trs& parent_local_transform = drive_joint_component->get_first_entity_ptr().get().get_component<TransformComponent>()->get_local_transform();

			const bool has_transform_drive = has_flag(limb_details.motor_drive->drive_type, MotorDriveType::TRANSFORM);
			const bool has_velocity_drive = has_flag(limb_details.motor_drive->drive_type, MotorDriveType::VELOCITY);

			if (has_transform_drive)
			{
				const trs& constraint_frame_in_actor1_local = drive_joint_component->get_second_local_frame();

				const trs constraint_frame_actor0_local = parent_local_transform * constraint_frame_in_actor0_local;
				const trs constraint_frame_target_local = limb_component->target_pose * constraint_frame_in_actor1_local;

				trs target_pose_in_constraint_space = invert(constraint_frame_actor0_local) * constraint_frame_target_local;
				target_pose_in_constraint_space.rotation = normalize(target_pose_in_constraint_space.rotation);

				if (physx::create_PxTransform(target_pose_in_constraint_space).isValid())
				{
					drive_joint_component->drive_transform = target_pose_in_constraint_space;
				}
				else
				{
					ASSERT(!physx::create_PxTransform(target_pose_in_constraint_space).isValid());
				}

				if(!has_velocity_drive)
				{
					drive_joint_component->angular_drive_velocity = vec3::zero;
					drive_joint_component->linear_drive_velocity = vec3::zero;
				}
			}

			if (has_velocity_drive)
			{
				const quat parent_constraint_frame_rotation = normalize(parent_local_transform.rotation * constraint_frame_in_actor0_local.rotation);

				const vec3 angular_velocity_constraint_space = conjugate(parent_constraint_frame_rotation) * angular_drive_velocity;
				drive_joint_component->angular_drive_velocity = angular_velocity_constraint_space * limb_details.motor_drive->velocity_drive_modifier;

				const vec3 desired_linear_velocity = delta_position / dt;

				const vec3 linear_velocity_constraint_space = conjugate(parent_constraint_frame_rotation) * desired_linear_velocity;
				drive_joint_component->linear_drive_velocity = linear_velocity_constraint_space * limb_details.motor_drive->velocity_drive_modifier;

				if (!has_transform_drive)
				{
					drive_joint_component->drive_transform = trs::identity;
				}
			}

			const float angular_damping = limb_component->calculate_desired_angular_damping(delta_angle);

			if (drive_joint_component->perform_slerp_drive)
			{
				drive_joint_component->slerp_drive_damping = angular_damping;
			}
			else
			{
				drive_joint_component->twist_drive_damping = angular_damping;
				drive_joint_component->swing_drive_damping = angular_damping;
			}

			const float linear_damping = limb_component->calculate_desired_linear_damping(length(delta_position));
			drive_joint_component->linear_drive_damping = linear_damping;
		}

		// Velocity drives for rotation stabilization.
		if (limb_details.drag_force.has_value())
		{
			const DragForceDetails& drag_details = limb_details.drag_force.value();
			DynamicBodyComponent* dynamic_body = limb.get_component<DynamicBodyComponent>();

			// Keyframe controller stage.
			{
				const vec3& raw_root_velocity = physical_animation_component->velocity;

				// Partial velocity drive.
				vec3 desired_velocity = delta_position / dt;
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

			vec3 desired_angular_velocity = angular_drive_velocity;
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

	void SimulationLimbState::on_exit()
	{
		ASSERT(!physical_animation_limb_component_ptr.is_empty());

		PhysicalAnimationLimbComponent* limb_component = static_cast<PhysicalAnimationLimbComponent*>(physical_animation_limb_component_ptr.get_for_write());

		PhysicalAnimationUtils::reset_motor_drive(limb_component);

		BaseLimbState::on_exit();
	}

}