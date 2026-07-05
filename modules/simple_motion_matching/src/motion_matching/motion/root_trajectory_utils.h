#pragma once 

#include "motion_matching_api.h"

#include "motion_matching/common.h"

#include <core/math.h>

namespace era_engine 
{
	namespace animation
	{
		class AnimationRootSampler;
	}

	class ERA_MOTION_MATCHING_API RootTrajectoryUtils
	{
	public:
		RootTrajectoryUtils() = delete;

		static float get_angular_velocity(const animation::AnimationRootSampler& sampler, float position);
		static float get_angular_acceleration(const animation::AnimationRootSampler& sampler, float position);

		static float get_linear_velocity(const animation::AnimationRootSampler& sampler, float position);
		static float get_linear_acceleration(const animation::AnimationRootSampler& sampler, float position);

		static trs get_trs_from_origin(const animation::AnimationRootSampler& sampler, float anim_position, float origin_anim_position);
	};
}