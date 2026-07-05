#pragma once

#include "motion_matching_api.h"

#include "motion_matching/features/motion_matching_feature.h"

#include <ecs/reflection.h>

namespace era_engine
{
	class ERA_MOTION_MATCHING_API PhaseFeature : public MotionMatchingFeature
	{
	public:
		PhaseFeature() = default;
		PhaseFeature(const PhaseFeature&) = default;
		~PhaseFeature() override;

		std::vector<float> compute_features(const FeatureComputationContext& context) const override;

		bool mark_animation(const animation::SkeletonComponent* skeleton_component, ref<animation::AnimationAssetClip> clip) const override;
		bool sample_animation(const animation::Skeleton* skeleton, ref<animation::AnimationAssetClip> clip, float sample_rate, std::vector<ref<MotionMatchingDatabase::Sample>>& out_samples) const override;

		ERA_VIRTUAL_REFLECT(MotionMatchingFeature)
	};
}