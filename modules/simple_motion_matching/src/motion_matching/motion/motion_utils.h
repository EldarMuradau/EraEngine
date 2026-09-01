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
            float dt,
            float gait_change_halflife = 0.1f);

        static vec3 desired_velocity_update(
            const vec3& input,
            const quat& rotation,
            float fwrd_speed,
            float side_speed,
            float back_speed);

        static quat desired_rotation_update(
            const quat& desired_rotation,
            bool has_input,
            float strafe_direction,
            bool desired_strafe,
            const vec3& desired_velocity);

        static void simulation_positions_update(
            vec3& position,
            vec3& velocity,
            vec3& acceleration,
            const vec3& desired_velocity,
            float halflife,
            float dt);

        static void simulation_rotations_update(
            quat& rotation,
            vec3& angular_velocity,
            const quat& desired_rotation,
            float halflife,
            float dt);
	};
}