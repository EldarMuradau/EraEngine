#pragma once 

#include "motion_matching_api.h"

#include "motion_matching/common.h"

#include <core/math.h>

namespace era_engine 
{
	class ERA_MOTION_MATCHING_API MotionUtils
	{
	public:
        MotionUtils() = delete;

        static void desired_gait_update(
            float& desired_gait,
            float& desired_gait_velocity,
            const float dt,
            const float gait_change_halflife = 0.1f);

        static vec3 desired_velocity_update(
            const vec3& input,
            const quat& simulation_rotation,
            const float fwrd_speed,
            const float side_speed,
            const float back_speed);

        static quat desired_rotation_update(
            const quat& desired_rotation,
            const vec3& input,
            const float strafe_direction,
            const bool desired_strafe,
            const vec3& desired_velocity);

        static void simulation_positions_update(
            vec3& position,
            vec3& velocity,
            vec3& acceleration,
            const vec3& desired_velocity,
            const float halflife,
            const float dt);

        static void simulation_rotations_update(
            quat& rotation,
            vec3& angular_velocity,
            const quat& desired_rotation,
            const float halflife,
            const float dt);
	};
}