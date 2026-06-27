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

		const PxDebugLine* lines = nullptr;

		size_t nb_lines = 0;

		PhysicsEngine::execute_read([&]() {
			const PxRenderBuffer* rb = &scene->getRenderBuffer();

			lines = rb->getLines();
			nb_lines = rb->getNbLines();
		});

		auto [vb, vertexPtr] = create_dynamic_vertex_buffer(sizeof(position_color), nb_lines * 2);
		auto [ib, indexPtr] = create_dynamic_index_buffer(sizeof(uint16), nb_lines * 2);

		position_color* vertices = (position_color*)vertexPtr;
		indexed_line16* render_lines = (indexed_line16*)indexPtr;

		for (uint32 i = 0; i < nb_lines; ++i)
		{
			const PxDebugLine& line = lines[i];

			*vertices++ = { create_vec3(line.pos0), vec3(1.f, 1.f, 1.f) };
			*vertices++ = { create_vec3(line.pos1), vec3(1.f, 1.f, 1.f) };

			*render_lines++ = { (uint16)(2 * i), (uint16)(2 * i + 1) };
		}

		renderDebug<debug_unlit_line_pipeline::position_color>(trs_to_mat4(trs::identity), vb, ib, vec4(1.f, 1.f, 1.f, 1.f), renderer_holder_rc->ldrRenderPass, true);
	}
}
