#include "motion_matching/motion/motion_system.h"
#include "motion_matching/motion_matching_database.h"
#include "motion_matching/motion_matching_component.h"
#include "motion_matching/trajectory/trajectory_component.h"
#include "motion_matching/motion/spring_motion_utils.h"
#include "motion_matching/motion/motion_component.h"
#include "motion_matching/motion/motion_utils.h"

#include "physics/cct_component.h"

#include "motion_matching/common.h"

#include "rendering/ecs/renderer_holder_root_component.h"
#include "rendering/debug_visualization.h"

#include "core/cpu_profiling.h"
#include "core/memory.h"
#include "core/string.h"
#include "core/ecs/input_receiver_component.h"
#include "core/debug/debug_var.h"

#include "engine/engine.h"

#include "ecs/update_groups.h"
#include "ecs/rendering/mesh_component.h"
#include "ecs/base_components/transform_component.h"

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
	static DebugVar<bool> draw_motion = DebugVar<bool>("motion_matching.debug_draw.draw_motion", false);

	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<MotionSystem>("MotionSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("motion_matching")))
			.method("update", &MotionSystem::update)(metadata("update_group", update_types::GAMEPLAY_BEFORE_PHYSICS_CONCURRENT))
			.method("update_base", &MotionSystem::update_base)(metadata("update_group", update_types::GAMEPLAY_BEFORE_PHYSICS_CONCURRENT))
			.method("reset_input", &MotionSystem::reset_input)(metadata("update_group", update_types::END))
			.method("debug_draw_update", &MotionSystem::debug_draw_update)(metadata("update_group", update_types::RENDER));
	}

	MotionSystem::MotionSystem(World* _world)
		: System(_world)
	{
		renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();
		ASSERT(renderer_holder_rc != nullptr);
	}

	MotionSystem::~MotionSystem()
	{
	}

	void MotionSystem::init()
	{
	}

	void MotionSystem::update(float dt)
	{
		using namespace animation;
		ZoneScopedN("MotionSystem::update");

        for (auto [handle, transform_component, motion_component]
			: world->group(components_group<TransformComponent, MotionComponent>).each())
        {
			MotionUtils::simulation_positions_update(
				motion_component.simulation_position,
				motion_component.velocity,
				motion_component.acceleration,
				motion_component.desired_velocity,
				motion_component.velocity_halflife,
				dt);

			MotionUtils::simulation_rotations_update(
				motion_component.simulation_rotation,
				motion_component.angular_velocity,
				motion_component.desired_rotation,
				motion_component.rotation_halflife,
				dt);

			if (physics::CharacterControllerComponent* cct_component = world->get_entity(handle).get_component_if_exists<physics::CharacterControllerComponent>())
			{
				cct_component->velocity = motion_component.velocity;
				transform_component.set_world_rotation(motion_component.simulation_rotation);
			}
			else
			{
				transform_component.set_world_transform(trs{ motion_component.simulation_position, motion_component.simulation_rotation, vec3(1.0f) });
			}
			motion_component.last_velocity = motion_component.velocity;
		}
	}

	void MotionSystem::update_base(float dt)
	{
		ZoneScopedN("MotionSystem::update_base");

		for (auto [handle, transform_component, motion_component, reciever_component] : world->group(components_group<TransformComponent, MotionComponent, InputReceiverComponent>).each())
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

		for (auto [handle, transform_component, motion_component] : world->group(components_group<TransformComponent, MotionComponent>).each())
		{
			motion_component.apply_desired_input();
		}
	}

	void MotionSystem::debug_draw_update(float dt)
	{
		ZoneScopedN("MotionSystem::debug_draw_update");

		if (draw_motion)
		{
			for (auto [handle, transform_component, motion_component] : world->group(components_group<TransformComponent, MotionComponent>).each())
			{
				renderWireSphere(motion_component.simulation_position, 0.05f, vec4(0.0f, 1.0f, 0.0f, 1.0f), renderer_holder_rc->ldrRenderPass);

				vec3 sim_dir_ = (
					motion_component.simulation_position + 0.6f * (motion_component.simulation_rotation * vec3(0.0f, 0.0f, 1.0f)));
				renderLine(motion_component.simulation_position, sim_dir_, vec4(0.0f, 1.0f, 0.0f, 1.0f), renderer_holder_rc->ldrRenderPass);
			}
		}
	}
}