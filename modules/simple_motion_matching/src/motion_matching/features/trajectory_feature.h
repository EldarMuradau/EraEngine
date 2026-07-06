#pragma once

#include "motion_matching_api.h"

#include "motion_matching/features/motion_matching_feature.h"

#include <ecs/reflection.h>

namespace era_engine
{
	class ERA_MOTION_MATCHING_API TrajectoryFeatureDesc : public FeatureDesc
	{
	public:
		TrajectoryFeatureDesc() = default;
		~TrajectoryFeatureDesc() override = default;

		FeatureDescType type = FeatureDescType::LOCATION;
		float time_offset = 0.0f;

		ERA_VIRTUAL_BINARY_SERIALIZE(FeatureDesc::name, type, time_offset);

		ERA_VIRTUAL_REFLECT(FeatureDesc)
	};

	class ERA_MOTION_MATCHING_API TrajectoryFeature : public MotionMatchingFeature
	{
	public:
		TrajectoryFeature() = default;
		TrajectoryFeature(const TrajectoryFeature&) = default;
		~TrajectoryFeature() override;

		uint32 get_feature_size() const override;

		std::vector<float> compute_features(const FeatureComputationContext& context) const override;

		bool mark_animation(const animation::SkeletonComponent* skeleton_component, ref<animation::AnimationAssetClip> clip) const override;
		bool sample_animation(const animation::Skeleton* skeleton, ref<animation::AnimationAssetClip> clip, float sample_rate, std::vector<ref<MotionMatchingDatabase::Sample>>& out_samples) const override;

		const static inline uint32 NUM_OF_TRAJECTORIES = 4;

		ERA_VIRTUAL_REFLECT(MotionMatchingFeature)
	};
}