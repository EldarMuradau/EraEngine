#pragma once 

#include "motion_matching_api.h"

#include "animation/animation_pose_sampler.h"
#include "animation/animation_clip.h"

#include <core/math.h>

namespace era_engine
{
	namespace animation
	{
		class Skeleton;
	}

	enum class RootMotionType : uint8
	{
		LOCATION,
		ROTATION,
		TRANSFORM
	};

	class ERA_MOTION_MATCHING_API RootMotion
	{
	public:
		RootMotion(const animation::Skeleton* _skeleton);

		const trs& get_initial_world_transform() const;
		void set_initial_world_transform(const trs& new_transform);

		RootMotionType get_motion_type() const;

		void attach_to_animation(const ref<animation::AnimationAssetClip>& animation, 
			float new_start_position, 
			float new_duration,
			RootMotionType new_type);
		const ref<animation::AnimationAssetClip>& get_current_animation() const;

		float get_current_time() const;
		void reset_time(float new_start_position, float new_duration);

		float get_start_position() const;

		float get_position_by_time(float time);
		float get_time_by_position(float position);

		void update_motion(float dt);

		// World space
		trs get_world_transform(float position) const;
		vec3 get_world_linear_velocity(float position) const;
		vec3 get_world_linear_acceleration(float position) const;
		vec3 get_world_angular_velocity(float position) const;

		// Local space
		float get_angular_velocity(float position) const;
		float get_angular_acceleration(float position) const;

		vec3 get_linear_velocity(float position) const;
		vec3 get_linear_acceleration(float position) const;

		trs get_transform(float position) const;

	private:
		animation::AnimationRootSampler root_sampler;

		const animation::Skeleton* skeleton = nullptr;

		trs initial_world_transform = trs::identity;

		float duration = 0.0f;
		float start_position = 0.0f;

		float current_time = 0.0f;
		float prev_time = 0.0f;

		RootMotionType type = RootMotionType::LOCATION;
	};
}