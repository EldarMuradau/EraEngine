#pragma once

#include "motion_matching_api.h"

#include "motion_matching/features/motion_matching_feature.h"

#include <animation/skeleton.h>

#include <ecs/reflection.h>

namespace era_engine
{
	class ERA_MOTION_MATCHING_API PoseFeatureDesc : public FeatureDesc
	{
	public:
		PoseFeatureDesc() = default;
		~PoseFeatureDesc() override = default;

		uint32 joint_id = INVALID_JOINT;
	};

	class ERA_MOTION_MATCHING_API PoseFeature : public MotionMatchingFeature
	{
	public:
		PoseFeature() = default;
		PoseFeature(const PoseFeature&) = default;
		~PoseFeature() override;

		void compute_features(const FeatureComputationContext& context) override;

		bool mark_animation(Entity entity, ref<animation::AnimationAssetClip> clip) const override;

		ERA_VIRTUAL_REFLECT(MotionMatchingFeature)
	};
}