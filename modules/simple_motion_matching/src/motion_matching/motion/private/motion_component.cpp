#include "motion_matching/motion/motion_component.h"

#include <ecs/entity.h>

#include <rttr/registration>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<MotionComponent>("MotionComponent")
			.constructor<ref<Entity::EcsData>>();
	}

	MotionComponent::MotionComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
	}

	MotionComponent::~MotionComponent()
	{
	}

	const vec3& MotionComponent::get_current_input() const
	{
		return current_input;
	}

	const vec3& MotionComponent::get_desired_input() const
	{
		return desired_input;
	}

	const vec3& MotionComponent::get_last_input() const
	{
		return last_input;
	}

	const vec3& MotionComponent::get_velocity() const
	{
		return velocity;
	}

	const vec3& MotionComponent::get_last_velocity() const
	{
		return last_velocity;
	}

	void MotionComponent::apply_input(const vec3& input, bool force)
	{
		desired_input += input;
		if (force)
		{
			apply_desired_input();
		}
	}

	void MotionComponent::apply_desired_input()
	{
		last_input = current_input;
		current_input = desired_input;
		desired_input = vec3::zero;
		applied_input_direction = vec3::zero;
	}

}