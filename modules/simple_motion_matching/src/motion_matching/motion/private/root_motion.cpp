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

	RootMotionType RootMotion::get_motion_type() const
	{
		return type;
	}

	void RootMotion::attach_to_animation(const ref<animation::AnimationAssetClip>& animation, 
		float new_start_position, 
		float new_duration,
		RootMotionType new_type)
	{
		type = new_type;
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

	vec3 RootMotion::get_world_linear_velocity(float position) const
	{
		const float frame_time = 1.0f / root_sampler.get_animation()->get_sample_rate();

		bool is_last_position = position + frame_time > root_sampler.get_duration();
		float current_position = is_last_position ? position - frame_time : position;
		float next_position = is_last_position ? position : position + frame_time;

		trs start_transform = initial_world_transform * root_sampler.sample_root(current_position).get_transform();
		trs target_transform = initial_world_transform * root_sampler.sample_root(next_position).get_transform();
		
		vec3 linear_velocity = (target_transform.position - start_transform.position) / frame_time;

		return linear_velocity;
	}

	vec3 RootMotion::get_world_linear_acceleration(float position) const
	{
		const float frame_time = 1.0f / root_sampler.get_animation()->get_sample_rate();

		bool is_last_position = position + frame_time > root_sampler.get_duration();
		float current_position = is_last_position ? position - frame_time : position;
		float next_position = is_last_position ? position : position + frame_time;

		vec3 current_velocity = get_world_linear_velocity(current_position);
		vec3 next_velocity = get_world_linear_velocity(next_position);

		return (next_velocity - current_velocity) / frame_time;
	}

	vec3 RootMotion::get_world_angular_velocity(float position) const
	{
		const float frame_time = 1.0f / root_sampler.get_animation()->get_sample_rate();

		bool is_last_position = position + frame_time > root_sampler.get_duration();
		float current_position = is_last_position ? position - frame_time : position;
		float next_position = is_last_position ? position : position + frame_time;

		trs start_transform = initial_world_transform * root_sampler.sample_root(current_position).get_transform();
		trs target_transform = initial_world_transform * root_sampler.sample_root(next_position).get_transform();

		vec3 angular_velocity = quat_differentiate_angular_velocity(target_transform.rotation, start_transform.rotation, frame_time);

		return angular_velocity;
	}

}