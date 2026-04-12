#include "motion_matching/features/phase_feature.h"

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<PhaseFeature>("PhaseFeature")
			.constructor<>()(policy::ctor::as_raw_ptr);
	}

	PhaseFeature::~PhaseFeature()
	{
	}

	std::vector<float> PhaseFeature::compute_features(const FeatureComputationContext& context)
	{
		// TODO:
		return  std::vector<float>{ 0.0f, 0.0f };
	}

	bool PhaseFeature::mark_animation(const animation::SkeletonComponent* skeleton_component, ref<animation::AnimationAssetClip> clip) const
	{
		// TODO:
		return true;
	}

	bool PhaseFeature::sample_animation(ref<animation::AnimationAssetClip> clip, float sample_rate, std::vector<ref<MotionMatchingDatabase::Sample>>& out_samples) const
	{
		// TODO:

		const uint32 num_samples = std::lrintf(1.0f / sample_rate);

		for (uint32 i = 0; i < num_samples; ++i)
		{
			ref<MotionMatchingDatabase::Sample>& sample = out_samples[i];
			sample->features.emplace_back(0.0f);
			sample->features.emplace_back(0.0f);
		}

		return true;
	}
}