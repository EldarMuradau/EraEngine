#include "motion_matching/motion_matching_anim_marker.h"
#include "motion_matching/features/motion_matching_feature.h"

#include "animation/animation_clip.h"
#include "animation/skeleton_component.h"

#include <asset/game_asset.h>
#include <asset/file_registry.h>

#include <core/math.h>

namespace era_engine
{
	bool MotionMatchingAnimMarker::mark_database(const animation::SkeletonComponent* skeleton_component, MotionMatchingDatabase& database)
	{
		using namespace animation;

		GameAssetsProvider provider;

		std::vector<JobHandle> handles;
		handles.reserve(database.animations.size());

		for (const ref<AnimationAssetClip>& animation : database.animations)
		{
			for (MotionMatchingFeature* feature : database.features)
			{
				const bool status = feature->mark_animation(skeleton_component, animation);
				ASSERT(status);
			}

			JobHandle save_job = provider.save_game_asset_to_file_async<AnimationAssetClip>(getPathFromAssetHandle(animation->handle), animation.get(), {}, false);
			handles.emplace_back(save_job);
		}

		for (JobHandle& handle : handles)
		{
			handle.wait_for_completion();
		}

		return true;
	}
}
