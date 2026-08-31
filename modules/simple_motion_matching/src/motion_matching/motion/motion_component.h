#pragma once

#include "motion_matching_api.h"

#include <core/math.h>

#include <ecs/component.h>

namespace era_engine
{
	class ERA_MOTION_MATCHING_API MotionComponent final : public Component
	{
	public:
		MotionComponent() = default;
		MotionComponent(ref<Entity::EcsData> _data);

		~MotionComponent() override;

		const vec3& get_current_input() const;
		const vec3& get_desired_input() const;
		const vec3& get_last_input() const;

		const vec3& get_velocity() const;
		const vec3& get_last_velocity() const;

		void apply_input(const vec3& input, bool force = false);

		ERA_VIRTUAL_REFLECT(Component)

	protected:
		void apply_desired_input();

	protected:
		vec3 current_input = vec3::zero;
		vec3 last_input = vec3::zero;
		vec3 desired_input = vec3::zero;

		vec3 applied_input_direction = vec3::zero;

		vec3 desired_velocity = vec3::zero;
		vec3 desired_velocity_change_curr = vec3::zero;
		vec3 desired_velocity_change_prev = vec3::zero;
		float desired_velocity_change_threshold = deg2rad(50.0);

		quat desired_rotation = quat::identity;
		vec3 desired_rotation_change_curr = vec3::zero;
		vec3 desired_rotation_change_prev = vec3::zero;
		float desired_rotation_change_threshold = deg2rad(50.0);

		float desired_gait = 0.0f;
		float desired_gait_velocity = 0.0f;

		vec3 velocity = vec3::zero;
		vec3 acceleration = vec3::zero;
		vec3 angular_velocity = vec3::zero;
		vec3 last_velocity = vec3::zero;

		float velocity_halflife = 0.27f;
		float rotation_halflife = 0.27f;

		float run_fwrd_speed = 5.0f;
		float run_side_speed = 4.0f;
		float run_back_speed = 3.5f;

		float walk_fwrd_speed = 2.25f;
		float walk_side_speed = 2.0f;
		float walk_back_speed = 1.75f;

		friend class MotionSystem;
		friend class TrajectoryMotionSystem;
	};
}