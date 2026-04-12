#include "motion_matching/motion_matching_component.h"
#include "motion_matching/motion_matching_database.h"
#include "motion_matching/features/motion_matching_feature_set.h"
#include "motion_matching/features/motion_matching_feature.h"

#include "motion_matching/features/pose_feature.h"
#include "motion_matching/features/phase_feature.h"
#include "motion_matching/features/trajectory_feature.h"

#include <animation/animation.h>

#include <rttr/registration>

namespace era_engine
{

	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<MotionMatchingComponent>("MotionMatchingComponent")
			.constructor<ref<Entity::EcsData>>();
	}

	MotionMatchingComponent::MotionMatchingComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{	
	}

	MotionMatchingComponent::~MotionMatchingComponent()
	{
	}

	SearchResult MotionMatchingComponent::search_animation(const MotionMatchingFeatureSet& feature_set, const std::string& database_id) const
	{
		using namespace animation;

		ref<MotionMatchingDatabase> database = MotionDatabaseRegistry::get_by_id(database_id);
		if (database == nullptr)
		{
			return SearchResult();
		}

		AnimationComponent* animation_component = get_entity().get_component<AnimationComponent>();

		SearchParams search_params;
		search_params.current_anim_position = animation_component->current_anim_position;
		search_params.current_animation = animation_component->current_animation;
		search_params.query = feature_set.get_all_feature_values();
		search_params.current_features = search_params.query;

		return database->search(search_params);
	}

}