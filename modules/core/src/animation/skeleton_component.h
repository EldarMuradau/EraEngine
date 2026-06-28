#pragma once 

#include "core_api.h"

#include "animation/skeleton.h"

#include "ecs/component.h"

#include <optional>

namespace era_engine::animation
{
	class ERA_CORE_API SkeletonComponent : public Component
	{
	public:
		SkeletonComponent(ref<Entity::EcsData> _data);
		~SkeletonComponent() override;

		SkeletonPose get_current_pose() const;

		void blend_local_transforms(const trs* local_transforms1, const trs* local_transforms2, float t, trs* out_blended_local_transforms) const;
		void get_skinning_matrices_from_local_transforms(mat4* out_skinning_matrices, const trs& world_transform = trs::identity) const;
		void get_skinning_matrices_from_local_transforms(trs* out_global_transforms, mat4* out_skinning_matrices, const trs& world_transform = trs::identity) const;
		void get_skinning_matrices_from_global_transforms(const trs* global_transforms, mat4* out_skinning_matrices) const;
		void get_skinning_matrices_from_global_transforms(const trs* global_transforms, mat4* out_skinning_matrices, const trs& world_transform) const;

		void set_joint_transform(const trs& new_transform, uint32 joint_id);
		void set_joint_rotation(const quat& new_rotation, uint32 joint_id);

		const trs& get_joint_transform(uint32 joint_id) const;
		const quat& get_joint_rotation(uint32 joint_id) const;

		void set_joint_translation(const vec3& new_translation, uint32 joint_id);
		const vec3& get_joint_translation(uint32 joint_id) const;

		void apply_pose(const SkeletonPose& pose);

		ERA_VIRTUAL_REFLECT(Component)

	public:
		ref<Skeleton> skeleton = nullptr;

		bool draw_sceleton = false;

	private:
		std::vector<JointTransform> local_transforms; // In parent space.

		friend class SkeletonUtils;
		friend class SkeletonSystem;
	};

	class ERA_CORE_API SkeletonUtils final
	{
		SkeletonUtils() = delete;

	public:
		static trs get_object_space_joint_transform(const SkeletonComponent* skeleton_component, uint32 joint_id, uint32 start_from = INVALID_JOINT);

		static trs get_object_space_joint_transform(const SkeletonPose& pose, const Skeleton* skeleton, uint32 joint_id, uint32 start_from = INVALID_JOINT);
	};
}