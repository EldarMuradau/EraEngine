#include "physics/physical_animation/ragdoll_drive_pose_sampler.h"
#include "physics/physical_animation/physical_animation_component.h"
#include "physics/physical_animation/physical_animation_utils.h"
#include "physics/physical_animation/ragdoll_profile.h"
#include "physics/physical_animation/dismemberment/ragdoll_dismemberment_component.h"

#include <animation/animation.h>

#include <core/debug/debug_var.h>
#include <core/traits.h>

#include <ecs/world.h>
#include <ecs/base_components/transform_component.h>

namespace era_engine::physics
{
    RagdollDrivePoseSampler::RagdollDrivePoseSampler()
    {
    }

    void RagdollDrivePoseSampler::sample_pose(PhysicalAnimationComponent* physical_animation_component,
        animation::SkeletonComponent* skeleton_to_update,
        const animation::SkeletonPose& pose,
        float dt) const
    {
        using namespace animation;

        if (!pose.is_valid())
        {
            return;
        }

        ref<Skeleton> skeleton = skeleton_to_update->skeleton;
        ASSERT(skeleton != nullptr);

        const uint32 root_id = 0;

        auto simulated_joints_end = physical_animation_component->simulated_joints.end();

        const float blend_value = physical_animation_component->get_world()->get_elapsed_fixed_update_share();

        for (const uint32 simulation_joint : physical_animation_component->simulated_joints_set)
        {
            if (simulation_joint == root_id)
            {
                continue;
            }

            const uint32 parent_id = skeleton->joints.at(simulation_joint).parent_id;
            const trs& parent_local = physical_animation_component->local_joint_poses[parent_id];
            const trs inverse_parent_local = invert(parent_local);

            trs limb_animation_pose = pose.get_joint_transform(simulation_joint).get_transform();
            limb_animation_pose.scale = vec3(1.0f);

            trs animation_object_space_pose = parent_local * limb_animation_pose;

            auto limb_iter = physical_animation_component->simulated_joints.find(simulation_joint);
            if (limb_iter == simulated_joints_end)
            {
                physical_animation_component->local_joint_poses[simulation_joint] = animation_object_space_pose;
            }
            else
            {
                Entity limb = limb_iter->second.get();

                PhysicalAnimationLimbComponent* limb_data_component = limb.get_component<PhysicalAnimationLimbComponent>();

                // A kinematic limb follows the drive target, which lags the animation by a frame. The mesh shows the animation
                // for it and fades to the physics pose when the limb starts simulating, so the start does not jump.
                const bool is_limb_simulated = limb_data_component->get_current_state_type() != PhysicalLimbStateType::KINEMATIC;
                const float limb_blend_time = is_limb_simulated ? physical_animation_component->blend_in_time : physical_animation_component->blend_out_time;
                const float limb_blend_step = limb_blend_time > 0.0f ? dt / limb_blend_time : 1.0f;

                float& mesh_physics_weight = limb_data_component->mesh_physics_weight;
                mesh_physics_weight = is_limb_simulated ? std::min(mesh_physics_weight + limb_blend_step, 1.0f) : std::max(mesh_physics_weight - limb_blend_step, 0.0f);

                PhysicalLimbBlendType result_type = limb_data_component->blend_type | limb_data_component->prev_blend_type;
                if (result_type == PhysicalLimbBlendType::NONE || mesh_physics_weight <= 0.0f)
                {
                    // By default use PURE_ANIMATION blend type.
                    result_type = PhysicalLimbBlendType::PURE_ANIMATION;
                }

                if (RagdollDismembermentLimbComponent* dismembered_limb = limb.get_component_if_exists<RagdollDismembermentLimbComponent>())
                {
                    if (dismembered_limb->is_dismembered())
                    {
                        mesh_physics_weight = 1.0f;
                        result_type = PhysicalLimbBlendType::PURE_PHYSICS;
                    }
                }

                if (result_type != PhysicalLimbBlendType::PURE_ANIMATION)
                {
                    const trs new_transform = sample_limb(limb,
                        limb_data_component,
                        physical_animation_component,
                        result_type,
                        animation_object_space_pose,
                        inverse_parent_local,
                        blend_value);

                    if (physical_animation_component->mesh_update_type == SkeletonUpdateType::ROTATION)
                    {
                        skeleton_to_update->set_joint_rotation(new_transform.rotation, simulation_joint);
                    }
                    else if (physical_animation_component->mesh_update_type == SkeletonUpdateType::TRANSLATION)
                    {
                        skeleton_to_update->set_joint_translation(new_transform.position, simulation_joint);
                    }
                    else if (physical_animation_component->mesh_update_type == SkeletonUpdateType::TRANSFORM)
                    {
                        skeleton_to_update->set_joint_transform(new_transform, simulation_joint);
                    }

                    trs new_local_child_transform = parent_local * new_transform;
                    new_local_child_transform.rotation = normalize(new_local_child_transform.rotation);

                    physical_animation_component->local_joint_poses[simulation_joint] = new_local_child_transform;
                }
                else
                {
                    physical_animation_component->local_joint_poses[simulation_joint] = animation_object_space_pose;
                }
            }
        }
    }

    void RagdollDrivePoseSampler::blend_with_transform(const trs& from_joint_transform, float blend_value, trs& out_transform) const
    {
        out_transform.position = lerp(from_joint_transform.position,
            out_transform.position,
            blend_value);

        if (!fuzzy_equals(from_joint_transform.rotation, out_transform.rotation))
        {
            out_transform.rotation = slerp(from_joint_transform.rotation,
                out_transform.rotation,
                blend_value);
        }
    }

    trs RagdollDrivePoseSampler::sample_limb(Entity limb,
        PhysicalAnimationLimbComponent* limb_data_component,
        PhysicalAnimationComponent* physical_animation_component,
        PhysicalLimbBlendType result_type,
        const trs& limb_animation_transform,
        const trs& inverse_parent_local_transform,
        float blend_time) const
    {
        const float blend_weight = physical_animation_component->blend_weight * limb_data_component->mesh_physics_weight;
        if (fuzzy_equals(blend_weight, 0.0f))
        {
            return inverse_parent_local_transform * limb_animation_transform;
        }

        trs new_object_pose = limb_data_component->physics_pose;

        if (!has_flag(result_type, PhysicalLimbBlendType::PURE_PHYSICS))
        {
            // Physics poses come at fixed rate, interpolate them always so the mesh does not step.
            blend_with_transform(limb_data_component->prev_physics_pose,
                blend_time,
                new_object_pose);

            if (has_flag(result_type, PhysicalLimbBlendType::BLEND_WITH_ANIMATION_POSE))
            {
                blend_with_transform(limb_animation_transform,
                    physical_animation_component->animation_blend_factor,
                    new_object_pose);
            }
        }

        if (!fuzzy_equals(blend_weight, 1.0f))
        {
            blend_with_transform(limb_animation_transform,
                blend_weight,
                new_object_pose);
        }

        new_object_pose.rotation = normalize(new_object_pose.rotation);

        return inverse_parent_local_transform * new_object_pose;
    }
}