#include "motion_matching/motion/motion_utils.h"
#include "motion_matching/motion/spring_motion_utils.h"

namespace era_engine
{
	void MotionUtils::desired_gait_update(float& desired_gait, float& desired_gait_velocity, float dt, float gait_change_halflife)
	{
		SpringMotionUtils::simple_spring_damper_exact(
			desired_gait,
			desired_gait_velocity,
			0.0f,
			gait_change_halflife,
			dt);
	}

	vec3 MotionUtils::desired_velocity_update(const vec3& input, const quat& rotation, float fwrd_speed, float side_speed, float back_speed)
	{
        // Scale stick by forward, sideways and backwards speeds
        vec3 local_desired_velocity = input.z < 0.0 ?
            vec3(side_speed, 0.0f, fwrd_speed) * input :
            vec3(side_speed, 0.0f, back_speed) * input;

        return rotation * local_desired_velocity;
    }

	quat MotionUtils::desired_rotation_update(const quat& desired_rotation, bool has_input, float strafe_direction, bool desired_strafe, const vec3& desired_velocity)
	{
        quat desired_rotation_curr = desired_rotation;

        // If strafe is active then desired direction is coming from right
        // stick as long as that stick is being used, otherwise we assume
        // forward facing
        if (desired_strafe)
        {
            vec3 desired_direction = quat(vec3::up, strafe_direction) * vec3(0, 0, -1);

            return quat(vec3::up, atan2f(desired_direction.x, desired_direction.z));
        }

        // If strafe is not active the desired direction comes from the left 
        // stick as long as that stick is being used
        else if (has_input)
        {
            vec3 desired_direction = normalize(desired_velocity);
            return quat(vec3::up, atan2f(desired_direction.x, desired_direction.z));
        }
        else
        {
            // Otherwise desired direction remains the same
            return desired_rotation_curr;
        }
    }

    // Taken from https://theorangeduck.com/page/spring-roll-call#controllers
	void MotionUtils::simulation_positions_update(vec3& position, vec3& velocity, vec3& acceleration, const vec3& desired_velocity, float halflife, float dt)
	{
        float y = halflife_to_damping(halflife) / 2.0f;
        vec3 j0 = velocity - desired_velocity;
        vec3 j1 = acceleration + j0 * y;
        float eydt = fast_negexpf(y * dt);

        vec3 position_prev = position;

        position = eydt * (((-j1) / (y * y)) + ((-j0 - j1 * dt) / y)) +
            (j1 / (y * y)) + j0 / y + desired_velocity * dt + position_prev;
        velocity = eydt * (j0 + j1 * dt) + desired_velocity;
        acceleration = eydt * (acceleration - j1 * y * dt);
	}

	void MotionUtils::simulation_rotations_update(quat& rotation, vec3& angular_velocity, const quat& desired_rotation, float halflife, float dt)
	{
        SpringMotionUtils::simple_spring_damper_exact(
            rotation,
            angular_velocity,
            desired_rotation,
            halflife,
            dt);
	}
}
