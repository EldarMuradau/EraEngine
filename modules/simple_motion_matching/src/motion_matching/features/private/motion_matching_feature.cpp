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

	void MotionMatchingFeature::compute_features(const FeatureComputationContext& context)
	{
	}

	void MotionMatchingFeature::store_features(std::vector<float>&& _values)
	{
		values = std::move(_values);
	}

	bool MotionMatchingFeature::mark_animation(Entity entity, ref<animation::AnimationAssetClip> clip) const
	{
		return false;
	}

	std::vector<float> MotionMatchingFeature::get_values() const
	{
		return values;
	}
	const std::vector<ref<FeatureDesc>>& MotionMatchingFeature::get_descriptors() const
	{
		return descriptors;
	}
}