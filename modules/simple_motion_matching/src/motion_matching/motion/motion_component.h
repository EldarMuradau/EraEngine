#pragma once

#include "motion_matching_api.h"
#include "motion_matching/motion/root_motion.h"

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

		bool has_root_motion() const;

		void init_root_motion(const animation::Skeleton* skeleton, 
			const ref<animation::AnimationAssetClip>& animation, 
			float start_position, 
			float duration,
			RootMotionType type);

		void reset_root_motion();

		const vec3& get_current_input() const;
		const vec3& get_desired_input() const;
		const vec3& get_last_input() const;

		const vec3& get_velocity() const;
		const vec3& get_last_velocity() const;

		void apply_input(const vec3& input, bool force = false);

		const quat& get_input_movement_rotation() const;
		void set_input_movement_rotation(const quat& new_input_rotation);

		ERA_VIRTUAL_REFLECT(Component)

	protected:
		void apply_desired_input();

	protected:
		vec3 current_input = vec3::zero;
		vec3 last_input = vec3::zero;
		vec3 desired_input = vec3::zero;

		quat input_movement_rotation = quat::identity;

		vec3 desired_velocity = vec3::zero;
		vec3 desired_velocity_change_curr = vec3::zero;
		vec3 desired_velocity_change_prev = vec3::zero;

		quat desired_rotation = quat::identity;
		vec3 desired_rotation_change_curr = vec3::zero;
		vec3 desired_rotation_change_prev = vec3::zero;

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
		float run_back_speed = 1.5f;

		float walk_fwrd_speed = 2.25f;
		float walk_side_speed = 1.5f;
		float walk_back_speed = 1.0f;

		std::optional<RootMotion> root_motion;

		friend class MotionSystem;
		friend class TrajectoryMotionSystem;
	};
}