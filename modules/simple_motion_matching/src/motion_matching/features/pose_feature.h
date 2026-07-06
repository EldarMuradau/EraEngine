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

		FeatureDescType type = FeatureDescType::LOCATION;
		FeatureDescBasis basis = FeatureDescBasis::XYZ;
		uint32 joint_id = INVALID_JOINT;

		ERA_VIRTUAL_BINARY_SERIALIZE(FeatureDesc::name, type, basis, joint_id);

		ERA_VIRTUAL_REFLECT(FeatureDesc)
	};

	class ERA_MOTION_MATCHING_API PoseFeature : public MotionMatchingFeature
	{
	public:
		PoseFeature() = default;
		PoseFeature(const PoseFeature&) = default;
		~PoseFeature() override;

		uint32 get_feature_size() const override;

		std::vector<float> compute_features(const FeatureComputationContext& context) const override;

		bool mark_animation(const animation::SkeletonComponent* skeleton_component, ref<animation::AnimationAssetClip> clip) const override;
		bool sample_animation(const animation::Skeleton* skeleton, ref<animation::AnimationAssetClip> clip, float sample_rate, std::vector<ref<MotionMatchingDatabase::Sample>>& out_samples) const override;

		ERA_VIRTUAL_REFLECT(MotionMatchingFeature)
	};
}