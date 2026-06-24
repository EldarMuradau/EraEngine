#include "motion_matching/motion_matching_anim_marker.h"
#include "motion_matching/features/motion_matching_feature.h"

#include "animation/animation_clip.h"
#include "animation/skeleton_component.h"

#include <core/math.h>

namespace era_engine
{
	bool MotionMatchingAnimMarker::mark_database(const animation::SkeletonComponent* skeleton_component, MotionMatchingDatabase& database)
	{
		using namespace animation;

		for (const ref<AnimationAssetClip>& animation : database.animations)
		{
			for (MotionMatchingFeature* feature : database.features)
			{
				const bool status = feature->mark_animation(skeleton_component, animation);
				ASSERT(status);
			}
		}

		uint32 anim_index = 0;
		const float timestep = 1.0f / database.sample_rate;

		for (const ref<AnimationAssetClip>& animation : database.animations)
		{
			const uint32 num_samples_per_animation = std::lrintf(database.sample_rate * animation->get_duration());

			std::vector<ref<MotionMatchingDatabase::Sample>> animation_samples;
			animation_samples.reserve(num_samples_per_animation);

			for (uint32 i = 0; i < num_samples_per_animation; ++i)
			{
				ref<MotionMatchingDatabase::Sample>& sample = animation_samples.emplace_back();
				sample->anim_index = anim_index;
				sample->anim_position = static_cast<float>(i) * timestep;
			}

			for (MotionMatchingFeature* feature : database.features)
			{
				const bool status = feature->sample_animation(animation, database.sample_rate, animation_samples);
				ASSERT(status);
			}

			database.samples.insert(database.samples.end(), std::make_move_iterator(animation_samples.begin()), std::make_move_iterator(animation_samples.end()));

			++anim_index;
		}

		return true;
	}
}
