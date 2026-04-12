#pragma once

#include "motion_matching_api.h"

#include "motion_matching/features/motion_matching_feature.h"

#include <ecs/reflection.h>

namespace era_engine
{
	class ERA_MOTION_MATCHING_API TrajectoryFeature : public MotionMatchingFeature
	{
	public:
		TrajectoryFeature() = default;
		TrajectoryFeature(const TrajectoryFeature&) = default;
		~TrajectoryFeature() override;

		void compute_features(const FeatureComputationContext& context) override;

		bool mark_animation(Entity entity, ref<animation::AnimationAssetClip> clip) const override;

		ERA_VIRTUAL_REFLECT(MotionMatchingFeature)
	};
}