#include "animation/skeleton_component.h"

#include "core/math.h"

#include <rttr/registration>

namespace era_engine::animation
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<SkeletonComponent>("SkeletonComponent")
			.constructor<ref<Entity::EcsData>>();
	}

	SkeletonComponent::SkeletonComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
	}

	SkeletonComponent::~SkeletonComponent()
	{
	}

	SkeletonPose SkeletonComponent::get_current_pose() const
	{
		return SkeletonPose(local_transforms);
	}

	void SkeletonComponent::set_joint_transform(const trs& new_transform, uint32 joint_id)
	{
		local_transforms[joint_id].set_transform(new_transform);
	}

	void SkeletonComponent::set_joint_rotation(const quat& new_rotaiton, uint32 joint_id)
	{
		local_transforms[joint_id].set_rotation(new_rotaiton);
	}

	const trs& SkeletonComponent::get_joint_transform(uint32 joint_id) const
	{
		return local_transforms[joint_id].get_transform();
	}

	const quat& SkeletonComponent::get_joint_rotation(uint32 joint_id) const
	{
		return local_transforms[joint_id].get_rotation();
	}

	void SkeletonComponent::set_joint_translation(const vec3& new_translation, uint32 joint_id)
	{
		local_transforms[joint_id].set_translation(new_translation);
	}

	const vec3& SkeletonComponent::get_joint_translation(uint32 joint_id) const
	{
		return local_transforms[joint_id].get_translation();
	}

	void SkeletonComponent::apply_pose(const SkeletonPose& pose)
	{
		local_transforms = pose.local_transforms;
	}

	void SkeletonComponent::blend_local_transforms(const trs* local_transforms1, const trs* local_transforms2, float t, trs* out_blended_local_transforms) const
	{
		t = clamp01(t);
		for (uint32 jointID = 0; jointID < (uint32)skeleton->joints.size(); ++jointID)
		{
			out_blended_local_transforms[jointID] = lerp(local_transforms1[jointID], local_transforms2[jointID], t);
		}
	}

	void SkeletonComponent::get_skinning_matrices_from_local_transforms(mat4* out_skinning_matrices, const trs& world_transform) const
	{
		uint32 num_joints = (uint32)skeleton->joints.size();
		trs* global_transforms = (trs*)alloca(sizeof(trs) * num_joints);

		for (uint32 i = 0; i < num_joints; ++i)
		{
			const SkeletonJoint& joint = skeleton->joints[i];
			const JointTransform& joint_transform = local_transforms[i];
			if (joint.parent_id != INVALID_JOINT && joint.parent_id < skeleton->joints.size())
			{
				ASSERT(i > joint.parent_id); // Parent already processed.
				global_transforms[i] = global_transforms[joint.parent_id] * joint_transform.get_transform();
			}
			else
			{
				global_transforms[i] = world_transform * joint_transform.get_transform();
			}

			out_skinning_matrices[i] = trs_to_mat4(global_transforms[i]) * skeleton->joints[i].inv_bind_transform;
		}
	}

	void SkeletonComponent::get_skinning_matrices_from_local_transforms(trs* out_global_transforms, mat4* out_skinning_matrices, const trs& world_transform) const
	{
		uint32 num_joints = (uint32)skeleton->joints.size();

		for (uint32 i = 0; i < num_joints; ++i)
		{
			const SkeletonJoint& joint = skeleton->joints[i];
			const JointTransform& joint_transform = local_transforms[i];

			if (joint.parent_id != INVALID_JOINT && joint.parent_id < skeleton->joints.size())
			{
				ASSERT(i > joint.parent_id); // Parent already processed
				out_global_transforms[i] = out_global_transforms[joint.parent_id] * joint_transform.get_transform();
			}
			else
			{
				out_global_transforms[i] = world_transform * joint_transform.get_transform();
			}

			out_skinning_matrices[i] = trs_to_mat4(out_global_transforms[i]) * skeleton->joints[i].inv_bind_transform;
		}
	}

	void SkeletonComponent::get_skinning_matrices_from_global_transforms(const trs* global_transforms, mat4* out_skinning_matrices) const
	{
		uint32 num_joints = (uint32)skeleton->joints.size();

		for (uint32 i = 0; i < num_joints; ++i)
		{
			out_skinning_matrices[i] = trs_to_mat4(global_transforms[i]) * skeleton->joints[i].inv_bind_transform;
		}
	}

	void SkeletonComponent::get_skinning_matrices_from_global_transforms(const trs* global_transforms, mat4* out_skinning_matrices, const trs& world_transform) const
	{
		uint32 num_joints = (uint32)skeleton->joints.size();

		for (uint32 i = 0; i < num_joints; ++i)
		{
			out_skinning_matrices[i] = trs_to_mat4(world_transform) * trs_to_mat4(global_transforms[i]) * skeleton->joints[i].inv_bind_transform;
		}
	}

	trs SkeletonUtils::get_object_space_joint_transform(const SkeletonComponent* skeleton_component, uint32 joint_id, uint32 start_from/* = INVALID_JOINT*/)
	{
		if (joint_id >= skeleton_component->local_transforms.size())
		{
			return trs::identity;
		}

		const ref<Skeleton>& skeleton = skeleton_component->skeleton;

		trs result = skeleton_component->local_transforms[joint_id].get_transform();
		uint32 parent_id = skeleton->joints[joint_id].parent_id;

		while (parent_id != INVALID_JOINT && parent_id != start_from && parent_id < skeleton->joints.size())
		{
			result = skeleton_component->local_transforms[parent_id].get_transform() * result;
			parent_id = skeleton->joints[parent_id].parent_id;
		}

		return result;
	}

	trs SkeletonUtils::get_object_space_joint_transform(const SkeletonPose& pose, const Skeleton* skeleton, uint32 joint_id, uint32 start_from/* = INVALID_JOINT*/)
	{
		if (joint_id >= pose.size())
		{
			return trs::identity;
		}

		trs result = pose.get_joint_transform(joint_id).get_transform();
		uint32 parent_id = skeleton->joints[joint_id].parent_id;

		while (parent_id != INVALID_JOINT && parent_id != start_from && parent_id < skeleton->joints.size())
		{
			result = pose.get_joint_transform(parent_id).get_transform() * result;
			parent_id = skeleton->joints[parent_id].parent_id;
		}

		return result;
	}
}