#include "motion_matching/motion/root_motion.h"
#include "motion_matching/motion/root_trajectory_utils.h"

namespace era_engine
{
	RootMotion::RootMotion(const animation::Skeleton* _skeleton)
		: skeleton(_skeleton)
	{
	}

	const trs& RootMotion::get_initial_world_transform() const
	{
		return initial_world_transform;
	}

	void RootMotion::set_initial_world_transform(const trs& new_transform)
	{
		initial_world_transform = new_transform;
	}

	void RootMotion::attach_to_animation(const ref<animation::AnimationAssetClip>& animation, float new_start_position, float new_duration)
	{
		root_sampler.init(skeleton, animation);

		reset_time(new_start_position, new_duration);
	}

	const ref<animation::AnimationAssetClip>& RootMotion::get_current_animation() const
	{
		return root_sampler.get_animation();
	}

	float RootMotion::get_angular_velocity(float position) const
	{
		return RootTrajectoryUtils::get_angular_velocity(root_sampler, position);
	}

	float RootMotion::get_angular_acceleration(float position) const
	{
		return RootTrajectoryUtils::get_angular_acceleration(root_sampler, position);
	}

	vec3 RootMotion::get_linear_velocity(float position) const
	{
		return RootTrajectoryUtils::get_linear_velocity(root_sampler, position);
	}

	vec3 RootMotion::get_linear_acceleration(float position) const
	{
		return RootTrajectoryUtils::get_linear_acceleration(root_sampler, position);
	}

	trs RootMotion::get_transform(float position) const
	{
		trs transform = root_sampler.sample_root(position).get_transform();
		return transform;
	}

	trs RootMotion::get_world_transform(float position) const
	{
		return initial_world_transform * get_transform(position);
	}

	float RootMotion::get_current_time() const
	{
		return current_time;
	}

	void RootMotion::reset_time(float new_start_position, float new_duration)
	{
		prev_time = 0.0f;
		current_time = 0.0f;

		start_position = new_start_position;
		duration = new_duration;
	}

	float RootMotion::get_start_position() const
	{
		return start_position;
	}

	float RootMotion::get_position_by_time(float time)
	{
		return clamp(start_position + time, 0.0f, start_position + duration);
	}

	float RootMotion::get_time_by_position(float position)
	{
		return clamp(position - start_position, 0.0f, duration);
	}

	void RootMotion::update_motion(float dt)
	{
		if (current_time >= duration)
		{
			return;
		}

		prev_time = current_time;
		current_time += dt;
	}
}
