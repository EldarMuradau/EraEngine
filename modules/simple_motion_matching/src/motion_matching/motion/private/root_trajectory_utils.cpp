#include "motion_matching/motion/root_trajectory_utils.h"

#include <animation/animation_pose_sampler.h>

namespace era_engine
{
	float RootTrajectoryUtils::get_angular_velocity(const animation::AnimationRootSampler& sampler, float position)
	{
		const float frame_time = 1.0f / sampler.get_animation()->get_sample_rate();

		bool is_last_position = position + frame_time > sampler.get_duration();
		float current_position = is_last_position ? position - frame_time : position;
		float next_position = is_last_position ? position : position + frame_time;

		trs next_to_current = get_trs_from_origin(sampler, current_position, next_position);
		float angular_velocity = get_yaw_angle(next_to_current.rotation) / frame_time;

		return angular_velocity;
	}

	float RootTrajectoryUtils::get_angular_acceleration(const animation::AnimationRootSampler& sampler, float position)
	{
		const float frame_time = 1.0f / sampler.get_animation()->get_sample_rate();

		bool is_last_position = position + frame_time > sampler.get_duration();
		float current_position = is_last_position ? position - frame_time : position;
		float next_position = is_last_position ? position : position + frame_time;

		float current_velocity = get_angular_velocity(sampler, current_position);
		float next_velocity = get_angular_velocity(sampler, next_position);

		return (next_velocity - current_velocity) / frame_time;
	}

	float RootTrajectoryUtils::get_linear_velocity(const animation::AnimationRootSampler& sampler, float position)
	{
		const float frame_time = 1.0f / sampler.get_animation()->get_sample_rate();

		bool is_last_position = position + frame_time > sampler.get_duration();
		float current_position = is_last_position ? position - frame_time : position;
		float next_position = is_last_position ? position : position + frame_time;

		trs next_to_current = get_trs_from_origin(sampler, current_position, next_position);
		float linear_velocity = length(next_to_current.position) / frame_time;

		return linear_velocity;
	}

	float RootTrajectoryUtils::get_linear_acceleration(const animation::AnimationRootSampler& sampler, float position)
	{
		const float frame_time = 1.0f / sampler.get_animation()->get_sample_rate();

		bool is_last_position = position + frame_time > sampler.get_duration();
		float current_position = is_last_position ? position - frame_time : position;
		float next_position = is_last_position ? position : position + frame_time;

		float current_velocity = get_linear_velocity(sampler, current_position);
		float next_velocity = get_linear_velocity(sampler, next_position);

		return (next_velocity - current_velocity) / frame_time;
	}

	trs RootTrajectoryUtils::get_trs_from_origin(const animation::AnimationRootSampler& sampler, float anim_position, float origin_anim_position)
	{
		const float frame_time = 1.0f / sampler.get_animation()->get_sample_rate();

		if (!sampler.is_valid())
		{
			return trs::identity;
		}

		trs start_to_zero = sampler.sample_root(anim_position).get_transform();
		trs target_to_zero = sampler.sample_root(origin_anim_position).get_transform();

		trs to_start_transform = invert(start_to_zero);
		return to_start_transform * target_to_zero;
	}
}
