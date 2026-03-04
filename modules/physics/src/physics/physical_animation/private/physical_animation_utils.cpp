#include "physics/physical_animation/physical_animation_utils.h"
#include "physics/core/physics.h"
#include "physics/body_component.h"
#include "physics/shape_component.h"
#include "physics/core/physics_utils.h"
#include "physics/shape_utils.h"
#include "physics/physical_animation/physical_animation_component.h"
#include "physics/joint.h"

#include <core/math.h>

#include <ecs/base_components/transform_component.h>

#include <animation/animation.h>

namespace era_engine::physics
{

	vec3 PhysicalAnimationUtils::calculate_delta_rotation_time_derivative(const quat& delta_rotation, float dt)
	{
		float delta_angle = 0.0f;
		vec3 delta_axis = vec3::zero;
		get_axis_rotation(delta_rotation, delta_axis, delta_angle);

		const vec3 desired_angular_velocity = normalize(delta_axis) * delta_angle / dt;
		return desired_angular_velocity;
	}

    bool PhysicalAnimationUtils::is_body_limb(RagdollLimbType type)
    {
        return type == RagdollLimbType::BODY_LOWER ||
            type == RagdollLimbType::BODY_MIDDLE ||
            type == RagdollLimbType::BODY_UPPER;
    }

    bool PhysicalAnimationUtils::is_head_limb(RagdollLimbType type)
    {
        return type == RagdollLimbType::HEAD;
    }

    bool PhysicalAnimationUtils::is_leg_limb(RagdollLimbType type)
    {
        return type == RagdollLimbType::LEG ||
            type == RagdollLimbType::CALF ||
            type == RagdollLimbType::FOOT;
    }

    bool PhysicalAnimationUtils::is_arm_limb(RagdollLimbType type)
    {
        return type == RagdollLimbType::ARM ||
            type == RagdollLimbType::FOREARM ||
            type == RagdollLimbType::HAND;
    }

    void PhysicalAnimationUtils::reset_motor_drive(PhysicalAnimationLimbComponent* limb_component)
    {
        ASSERT(limb_component != nullptr);

        if (limb_component->drive_joint_component.is_empty())
        {
            return;
        }

        D6JointComponent* drive_joint_component = dynamic_cast<D6JointComponent*>(limb_component->drive_joint_component.get_for_write());

        drive_joint_component->drive_transform = trs::identity;

        drive_joint_component->linear_drive_damping = limb_component->linear_damping_range.y * RagdollStrengthConfig::LINEAR_DAMPING_MODIFIER;

        if (drive_joint_component->perform_slerp_drive)
        {
            drive_joint_component->slerp_drive_damping = limb_component->angular_damping_range.y * RagdollStrengthConfig::ANGULAR_DAMPING_MODIFIER;
        }
        else
        {
            drive_joint_component->swing_drive_damping = limb_component->angular_damping_range.y * RagdollStrengthConfig::ANGULAR_DAMPING_MODIFIER;
            drive_joint_component->twist_drive_damping = limb_component->angular_damping_range.y * RagdollStrengthConfig::ANGULAR_DAMPING_MODIFIER;
        }
    }

    void PhysicalAnimationUtils::set_simulation_for_limb(const PhysicalAnimationLimbComponent* limb_component,
        bool enable_simulation,
        bool enable_gravity /*= false*/)
    {
        ASSERT(limb_component != nullptr);

        DynamicBodyComponent* dynamic_body_component = limb_component->get_entity().get_component<DynamicBodyComponent>();
        ASSERT(dynamic_body_component != nullptr);

        dynamic_body_component->use_gravity = enable_gravity;

        dynamic_body_component->simulated = enable_simulation;

        if (!enable_simulation)
        {
            dynamic_body_component->simulated = false;
            dynamic_body_component->linear_velocity = vec3::zero;
            dynamic_body_component->angular_velocity = vec3::zero;
        }

        if (limb_component->drive_joint_component.get() != nullptr)
        {
            const D6JointComponent* drive_joint_component = dynamic_cast<const D6JointComponent*>(limb_component->drive_joint_component.get());

            drive_joint_component->get_first_entity_ptr().get().get_component<DynamicBodyComponent>()->simulated = enable_simulation;
        }
    }

    void PhysicalAnimationUtils::force_sync_limb_to_skeleton(PhysicalAnimationLimbComponent* limb_component,
        const animation::SkeletonComponent* skeleton_component,
        const trs& ragdoll_world_transform)
    {
        using namespace animation;

        ASSERT(limb_component != nullptr);
        ASSERT(skeleton_component != nullptr);

        Entity limb = limb_component->get_entity();

        TransformComponent* limb_transform = limb.get_component<TransformComponent>();
        ASSERT(limb_transform != nullptr);

        AnimationComponent* limb_animation = skeleton_component->get_entity().get_component<AnimationComponent>();
        ASSERT(limb_animation != nullptr);

        trs limb_object_space_pose = SkeletonUtils::get_object_space_joint_transform(limb_animation->current_animation_pose, 
            skeleton_component->skeleton.get(),
            limb_component->joint_id);
        limb_object_space_pose.scale = vec3(1.0f);

        limb_component->prev_physics_pose = limb_object_space_pose;
        limb_component->physics_pose = limb_object_space_pose;

        PhysicsUtils::manual_set_physics_transform(limb, ragdoll_world_transform * limb_object_space_pose, true);
    }
}