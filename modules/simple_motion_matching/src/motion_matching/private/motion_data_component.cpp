#include "motion_matching/motion_data_component.h"
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
		registration::class_<MotionDataComponent>("MotionDataComponent")
			.constructor<ref<Entity::EcsData>>();
	}

	MotionDataComponent::MotionDataComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
		
	}

	MotionDataComponent::~MotionDataComponent()
	{
	}
}