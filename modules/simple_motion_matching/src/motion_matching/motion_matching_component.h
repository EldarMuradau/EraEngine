#pragma once

#include "motion_matching_api.h"

#include "motion_matching/common.h"

#include <ecs/component.h>

namespace era_engine
{
	class MotionMatchingFeatureSet;

	class ERA_MOTION_MATCHING_API MotionMatchingComponent final : public Component
	{
	public:
		MotionMatchingComponent() = default;
		MotionMatchingComponent(ref<Entity::EcsData> _data);

		~MotionMatchingComponent() override;

		SearchResult search_animation(const MotionMatchingFeatureSet& feature_set, const std::string& database_id) const;

		ERA_VIRTUAL_REFLECT(Component)
	};
}