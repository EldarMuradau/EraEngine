#include "physics/physical_animation/drive_pose_sampler.h"
#include "physics/physical_animation/physical_animation_component.h"
#include "physics/physical_animation/physical_animation_utils.h"
#include "physics/physical_animation/ragdoll_profile.h"

#include <animation/animation.h>

#include <core/debug/debug_var.h>
#include <core/traits.h>

#include <ecs/world.h>
#include <ecs/base_components/transform_component.h>

namespace era_engine::physics
{
	DrivePoseSampler::DrivePoseSampler()
	{
	}

	void DrivePoseSampler::sample_pose(PhysicalAnimationComponent* physical_animation_component,
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

		const float fixed_update_dt = physical_animation_component->get_world()->get_fixed_update_dt();

		const uint32 root_id = 0;
		trs root_local_transform = SkeletonUtils::get_object_space_joint_transform(pose, skeleton.get(), root_id);
		root_local_transform.scale = vec3(1.0f);

		physical_animation_component->local_joint_poses[root_id] = root_local_transform;

		auto simulated_joints_end = physical_animation_component->simulated_joints.end();

		if (!physical_animation_component->reached_physics_pose)
		{
			physical_animation_component->elapsed_blend_time += dt;
		}

		const float blend_value = physical_animation_component->reached_physics_pose ? 1.0f : std::min(physical_animation_component->elapsed_blend_time / fixed_update_dt, 1.0f);
		if (physical_animation_component->elapsed_blend_time >= fixed_update_dt)
		{
			physical_animation_component->reached_physics_pose = true;
		}

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

			auto limb_iter = physical_animation_component->simulated_joints.find(simulation_joint);
			if (limb_iter == simulated_joints_end)
			{
				physical_animation_component->local_joint_poses[simulation_joint] = parent_local * limb_animation_pose;
			}
			else
			{
				Entity limb = limb_iter->second.get();

				PhysicalAnimationLimbComponent* limb_data_component = limb.get_component<PhysicalAnimationLimbComponent>();

				// Combine blend types to process ragdoll profile transition.
				PhysicalLimbBlendType result_type = limb_data_component->blend_type | limb_data_component->prev_blend_type;
				if (result_type == PhysicalLimbBlendType::NONE)
				{
					// By default use PURE_ANIMATION blend type.
					result_type = PhysicalLimbBlendType::PURE_ANIMATION;
				}

				if (result_type != PhysicalLimbBlendType::PURE_ANIMATION)
				{
					const trs new_transform = sample_limb(limb,
						limb_data_component,
						physical_animation_component,
						result_type,
						limb_animation_pose,
						inverse_parent_local,
						blend_value);

					if (physical_animation_component->mesh_blend_type == SkeletalMeshBlendType::ROTATION)
					{
						skeleton->set_joint_rotation(new_transform.rotation, simulation_joint);
					}
					else if (physical_animation_component->mesh_blend_type == SkeletalMeshBlendType::TRANSLATION)
					{
						skeleton->set_joint_translation(new_transform.position, simulation_joint);
					}
					else if (physical_animation_component->mesh_blend_type == SkeletalMeshBlendType::TRANSFORM)
					{
						skeleton->set_joint_transform(new_transform, simulation_joint);
					}

					trs new_local_child_transform = parent_local * new_transform;
					new_local_child_transform.rotation = normalize(new_local_child_transform.rotation);

					physical_animation_component->local_joint_poses[simulation_joint] = new_local_child_transform;
				}
				else
				{
					physical_animation_component->local_joint_poses[simulation_joint] = parent_local * limb_animation_pose;
				}
			}
		}
	}

	void DrivePoseSampler::blend_with_transform(const trs& from_joint_transform, float blend_value, trs& out_transform) const
	{
		if (fuzzy_equals(from_joint_transform.position, vec3::zero) || 
			fuzzy_equals(from_joint_transform.rotation, quat::identity))
		{
			return;
		}

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

	trs DrivePoseSampler::sample_limb(Entity limb,
		PhysicalAnimationLimbComponent* limb_data_component,
		PhysicalAnimationComponent* physical_animation_component,
		PhysicalLimbBlendType result_type,
		const trs& limb_animation_transform,
		const trs& inverse_parent_local_transform,
		float blend_time) const
	{
		if (fuzzy_equals(physical_animation_component->blend_weight, 0.0f))
		{
			return limb_animation_transform;
		}

		trs new_transform = inverse_parent_local_transform * limb_data_component->physics_pose;
		if (!has_flag(result_type, PhysicalLimbBlendType::PURE_PHYSICS))
		{
			if (has_flag(result_type, PhysicalLimbBlendType::BLEND_WITH_ANIMATION_POSE))
			{
				blend_with_transform(limb_animation_transform, physical_animation_component->blend_factor, new_transform);
			}

			if (has_flag(result_type, PhysicalLimbBlendType::BLEND_WITH_PREV_POSE))
			{
				const trs prev_local_transform = inverse_parent_local_transform * limb_data_component->prev_physics_pose;
				blend_with_transform(prev_local_transform, blend_time, new_transform);
			}
		}

		if (!fuzzy_equals(physical_animation_component->blend_weight, 1.0f))
		{
			// Blend with animation step.
			blend_with_transform(limb_animation_transform, physical_animation_component->blend_weight, new_transform);
		}

		new_transform.rotation = normalize(new_transform.rotation);

		return new_transform;
	}
}