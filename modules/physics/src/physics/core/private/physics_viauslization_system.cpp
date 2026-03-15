#include "physics/core/physics_visualization_system.h"
#include "physics/core/physics.h"

#include <ecs/update_groups.h>

#include <core/debug/debug_var.h>

#include <rendering/ecs/renderer_holder_root_component.h>
#include <rendering/debug_visualization.h>

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine::physics
{
	static DebugVar<bool> visualize_shapes = DebugVar<bool>("physics.visualize_shapes", false);

	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<PhysicsVisualizationSystem>("PhysicsVisualizationSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("physics")))
			.method("update", &PhysicsVisualizationSystem::update)(metadata("update_group", update_types::RENDER), metadata("After", std::vector<std::string>{"AnimationSystem::update"}));
	}

	PhysicsVisualizationSystem::PhysicsVisualizationSystem(World* _world)
		: System(_world)
	{
	}

	void PhysicsVisualizationSystem::init()
	{
		renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();
		ASSERT(renderer_holder_rc != nullptr);
	}

	void PhysicsVisualizationSystem::update(float dt)
	{
		using namespace physx;

		if (!visualize_shapes)
		{
			return;
		}

		auto scene = physics::PhysicsEngine::get_physics_core()->get_scene();

		const PxRenderBuffer* rb = nullptr;
		const PxDebugLine* lines = nullptr;

		size_t nb_lines = 0;

		PhysicsEngine::execute_read([&]() {
			rb = &scene->getRenderBuffer();

			for (PxU32 i = 0; i < rb->getNbPoints(); i++)
			{
				const PxDebugPoint& point = rb->getPoints()[i];
				renderPoint(create_vec3(point.pos), vec4(1.0f), renderer_holder_rc->ldrRenderPass, true);
			}

			lines = rb->getLines();
			nb_lines = rb->getNbLines();
		});

		for (PxU32 i = 0; i < nb_lines; i++)
		{
			const PxDebugLine& line = lines[i];
			renderLine(create_vec3(line.pos0), create_vec3(line.pos1), vec4(1.0f), renderer_holder_rc->ldrRenderPass, true);
		}
	}
}
