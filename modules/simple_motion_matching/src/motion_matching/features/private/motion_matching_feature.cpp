#include "motion_matching/features/motion_matching_feature.h"

#include "motion_matching/trajectory/trajectory_component.h"
#include "motion_matching/motion/motion_component.h"

#include "animation/animation.h"
#include "animation/skeleton_component.h"

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<FeatureDesc>("FeatureDesc")
			.constructor<>()(policy::ctor::as_raw_ptr);

		registration::class_<MotionMatchingFeature>("MotionMatchingFeature")
			.constructor<>()(policy::ctor::as_raw_ptr);
	}

	void FeatureComputationContext::fill_context(Entity _entity, float _dt)
	{
		using namespace animation;

		entity = _entity;
		dt = _dt;

		world = entity.get_world();
		
		skeleton_component = entity.get_component<SkeletonComponent>();
		animation_component = entity.get_component<AnimationComponent>();
		
		trajectory_component = entity.get_component<TrajectoryComponent>();
		motion_component = entity.get_component<MotionComponent>();
	}

	MotionMatchingFeature::~MotionMatchingFeature()
	{
	}

	std::vector<float> MotionMatchingFeature::compute_features(const FeatureComputationContext& context) const
	{
		return  std::vector<float>{};
	}

	bool MotionMatchingFeature::mark_animation(const animation::SkeletonComponent* skeleton_component, ref<animation::AnimationAssetClip> clip) const
	{
		return true;
	}

	bool MotionMatchingFeature::sample_animation(const animation::Skeleton* skeleton, ref<animation::AnimationAssetClip> clip, float sample_rate, std::vector<ref<MotionMatchingDatabase::Sample>>& out_samples) const
	{
		return true;
	}

	const std::vector<ref<FeatureDesc>>& MotionMatchingFeature::get_descriptors() const
	{
		return descriptors;
	}

	uint32 MotionMatchingFeature::get_feature_size() const
	{
		return 0;
	}
}