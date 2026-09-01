#include "motion_matching/motion/motion_system.h"
#include "motion_matching/motion_matching_database.h"
#include "motion_matching/motion_matching_component.h"
#include "motion_matching/trajectory/trajectory_component.h"
#include "motion_matching/motion/spring_motion_utils.h"
#include "motion_matching/motion/motion_component.h"
#include "motion_matching/motion/motion_utils.h"

#include "physics/cct_component.h"

#include "motion_matching/common.h"

#include "core/cpu_profiling.h"
#include "core/memory.h"
#include "core/string.h"
#include "core/ecs/input_receiver_component.h"

#include "engine/engine.h"

#include "ecs/update_groups.h"
#include "ecs/rendering/mesh_component.h"
#include "ecs/base_components/transform_component.h"

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<MotionSystem>("MotionSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("motion_matching")))
			.method("update", &MotionSystem::update)(metadata("update_group", update_types::GAMEPLAY_BEFORE_PHYSICS))
			.method("update_base", &MotionSystem::update_base)(metadata("update_group", update_types::GAMEPLAY_NORMAL_CONCURRENT))
			.method("reset_input", &MotionSystem::reset_input)(metadata("update_group", update_types::END_FIXED));
	}

	MotionSystem::MotionSystem(World* _world)
		: System(_world)
	{
	}

	MotionSystem::~MotionSystem()
	{
	}

	void MotionSystem::init()
	{
	}

	void MotionSystem::update(float dt)
	{
		ZoneScopedN("MotionSystem::update");

        for (auto&& [handle, transform_component, reciever_component, motion_component]
			: world->group(components_group<TransformComponent, InputReceiverComponent, MotionComponent>).each())
        {
			trs desired_world_transform = transform_component.get_world_transform();

			// Get gamepad stick states
			const vec3 input = noz(motion_component.get_desired_input());

			// Get if strafe is desired
			bool desired_strafe = reciever_component.get_frame_input().keyboard[key_ctrl].down;

			const float strafe_direction = 0.0f;

			// Get the desired gait (walk / run)
			MotionUtils::desired_gait_update(
				motion_component.desired_gait,
				motion_component.desired_gait_velocity,
				dt);

			// Get the desired simulation speeds based on the gait
			float simulation_fwrd_speed = lerpf(motion_component.run_fwrd_speed, motion_component.walk_fwrd_speed, motion_component.desired_gait);
			float simulation_side_speed = lerpf(motion_component.run_side_speed, motion_component.walk_side_speed, motion_component.desired_gait);
			float simulation_back_speed = lerpf(motion_component.run_back_speed, motion_component.walk_back_speed, motion_component.desired_gait);

			// Get the desired velocity
			vec3 desired_velocity_curr = MotionUtils::desired_velocity_update(
				input,
				motion_component.input_movement_rotation,
				simulation_fwrd_speed,
				simulation_side_speed,
				simulation_back_speed);

			// Get the desired rotation/direction
			quat desired_rotation_curr = MotionUtils::desired_rotation_update(
				desired_world_transform.rotation,
				length(input) > 0.01f,
				strafe_direction,
				desired_strafe,
				desired_velocity_curr);

			motion_component.desired_velocity_change_prev = motion_component.desired_velocity_change_curr;
			motion_component.desired_velocity_change_curr = (desired_velocity_curr - motion_component.desired_velocity) / dt;
			motion_component.desired_velocity = desired_velocity_curr;

			motion_component.desired_rotation_change_prev = motion_component.desired_rotation_change_curr;
			motion_component.desired_rotation_change_curr = quat_to_scaled_angle_axis(abs((conjugate(desired_rotation_curr) * motion_component.desired_rotation))) / dt;
			motion_component.desired_rotation = desired_rotation_curr;

			MotionUtils::simulation_positions_update(
				desired_world_transform.position,
				motion_component.velocity,
				motion_component.acceleration,
				motion_component.desired_velocity,
				motion_component.velocity_halflife,
				dt);

			MotionUtils::simulation_rotations_update(
				desired_world_transform.rotation,
				motion_component.angular_velocity,
				motion_component.desired_rotation,
				motion_component.rotation_halflife,
				dt);

			if (physics::CharacterControllerComponent* cct_component = world->get_entity(handle).get_component_if_exists<physics::CharacterControllerComponent>())
			{
				vec3& raw_cct_velocity = cct_component->velocity.get_for_write();
				raw_cct_velocity.x = motion_component.velocity.x;
				raw_cct_velocity.z = motion_component.velocity.z;
				transform_component.set_world_rotation(desired_world_transform.rotation);
			}
			else
			{
				transform_component.set_world_transform(desired_world_transform);
			}
			motion_component.last_velocity = motion_component.velocity;
		}
	}

	void MotionSystem::update_base(float dt)
	{
		ZoneScopedN("MotionSystem::update_base");

		for (auto&& [handle, transform_component, motion_component, reciever_component] : world->group(components_group<TransformComponent, MotionComponent, InputReceiverComponent>).each())
		{
			Entity movable = world->get_entity(handle);

			const UserInput& user_input = reciever_component.get_frame_input();
			const vec3& input = reciever_component.get_current_input();

			motion_component.apply_input(input);
		}
	}

	void MotionSystem::reset_input(float dt)
	{
		ZoneScopedN("MotionSystem::reset_input");

		for (auto&& [handle, transform_component, motion_component] : world->group(components_group<TransformComponent, MotionComponent>).each())
		{
			motion_component.apply_desired_input();
		}
	}
}