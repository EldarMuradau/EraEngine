#pragma once

#include "motion_matching_api.h"

#include "motion_matching/common.h"
#include "motion_matching/features/motion_matching_feature_set.h"

#include <ecs/component.h>

namespace era_engine
{
	class FeatureComputationContext;

	class ERA_MOTION_MATCHING_API MotionDataComponent : public Component
	{
	public:
		MotionDataComponent() = default;
		MotionDataComponent(ref<Entity::EcsData> _data);

		~MotionDataComponent() override;

		std::function<void(float)> on_update_func;
		std::function<void()> on_search_time_func;
		std::function<void()> on_search_recuperation_func;

		std::function<bool()> need_force_start_search_func;

		std::function<MotionMatchingFeatureSet(const FeatureComputationContext&)> calculate_feature_set_func;
		std::function<std::string()> get_motion_database_id_func;

		std::function<void(const SearchResult&)> on_search_succeeded_func;
		std::function<void()> on_search_failed_func;

		ERA_VIRTUAL_REFLECT(Component)

	public:
		float search_time = 0.1f;
		float search_timer = search_time;
	};
}