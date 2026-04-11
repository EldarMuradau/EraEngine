#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"

#include <ecs/component.h>

#include <core/math.h>
#include <core/memory.h>

#include <dx/dx_buffer.h>
#include <geometry/mesh_builder.h>

namespace era_engine
{
	class ClothRenderComponent;
}

namespace era_engine::physics
{
	class ERA_PHYSICS_API PBDClothComponent : public Component
	{
	public:
		PBDClothComponent() = default;
		PBDClothComponent(ref<Entity::EcsData> _data);

		vec3* get_positions() const;

		void sync_positions_buffers_locked();

		std::tuple<dx_vertex_buffer_group_view, dx_vertex_buffer_group_view, dx_index_buffer_view, SubmeshInfo> get_render_data(ClothRenderComponent* render_component);

		// Simple cloth setup.
		uint32 num_x = 0;
		uint32 num_z = 0;

	private:
		vec3* view_pos_buffer = nullptr;
		physx::PxVec4* native_pos_buffer = nullptr;
		physx::PxU32 num_particles = 0;

	private:
		Allocator allocator;

		physx::PxPBDMaterial* material = nullptr;
		physx::PxPBDParticleSystem* particle_system = nullptr;
		physx::PxParticleClothBuffer* cloth_buffer = nullptr;
	};
}