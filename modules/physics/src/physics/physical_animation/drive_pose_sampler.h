#pragma once

#include "physics_api.h"

#include "physics/physical_animation/ragdoll_profile.h"

#include <core/math.h>
#include <ecs/entity.h>

namespace era_engine::animation
{
    class SkeletonComponent;
    class SkeletonPose;
}

namespace era_engine::physics
{
    class PhysicalAnimationComponent;
	class PhysicalAnimationLimbComponent;

	class ERA_PHYSICS_API DrivePoseSampler
	{
	public:
        DrivePoseSampler();

        void sample_pose(PhysicalAnimationComponent* physical_animation_component,
            animation::SkeletonComponent* skeleton_to_update,
            const animation::SkeletonPose& pose,
			float dt) const;

	private:
		void blend_with_transform(const trs& from_joint_transform,
			float blend_value,
			trs& out_transform) const;

		trs sample_limb(Entity limb,
			PhysicalAnimationLimbComponent* limb_data_component,
			PhysicalAnimationComponent* physical_animation_component,
			PhysicalLimbBlendType result_type,
			const trs& limb_animation_transform,
			const trs& inverse_parent_local_transform,
			float blend_time) const;
	};
}