#pragma once 

#include "motion_matching_api.h"

#include "motion_matching/motion_matching_database.h"

namespace era_engine 
{
	namespace animation
	{
		class SkeletonComponent;
	}

	class ERA_MOTION_MATCHING_API MotionMatchingAnimMarker final
	{
	public:
		static bool mark_database(const animation::SkeletonComponent* skeleton_component, MotionMatchingDatabase& database);
	};
}