#include "physics/pbd/pbd_cloth_component.h"
#include "physics/core/physics.h"

#include <ecs/rendering/cloth_render_component.h>

#include <animation/skinning.h>

#include <dx/dx_context.h>

namespace era_engine::physics
{
	PBDClothComponent::PBDClothComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
	}

	vec3* PBDClothComponent::get_positions() const
	{
		return view_pos_buffer;
	}

	void PBDClothComponent::sync_positions_buffers_locked()
	{
		using namespace physx;

		PxCudaContextManager* cuda_context_manager = PhysicsEngine::get_physics_core()->get_cuda_context_manager();

		PxScopedCudaLock lock{ *cuda_context_manager };

		PxVec4* positions_inv_masses = cloth_buffer->getPositionInvMasses();

		const PxU32 num_particles = cloth_buffer->getNbActiveParticles();
		cuda_context_manager->acquireContext();

		PxCudaContext* cuda_context = cuda_context_manager->getCudaContext();
		cuda_context->memcpyDtoH(native_pos_buffer, CUdeviceptr(positions_inv_masses), sizeof(PxVec4) * num_particles);

		cuda_context_manager->releaseContext();

		for (uint32 i = 0; i < num_x * num_z; i += 4)
		{
			view_pos_buffer[i + 0] = vec3(native_pos_buffer[i + 0].x, native_pos_buffer[i + 0].y, native_pos_buffer[i + 0].z);
			view_pos_buffer[i + 1] = vec3(native_pos_buffer[i + 1].x, native_pos_buffer[i + 1].y, native_pos_buffer[i + 1].z);
			view_pos_buffer[i + 2] = vec3(native_pos_buffer[i + 2].x, native_pos_buffer[i + 2].y, native_pos_buffer[i + 2].z);
			view_pos_buffer[i + 3] = vec3(native_pos_buffer[i + 3].x, native_pos_buffer[i + 3].y, native_pos_buffer[i + 3].z);
		}
	}

	std::tuple<dx_vertex_buffer_group_view, dx_vertex_buffer_group_view, dx_index_buffer_view, SubmeshInfo> PBDClothComponent::get_render_data(ClothRenderComponent* render_component)
	{
		uint32 num_vertices = num_x * num_z;
		uint32 num_triangles = (num_x - 1) * (num_z - 1) * 2;

		auto [position_vertex_buffer, position_ptr] = create_dynamic_vertex_buffer(sizeof(vec3), num_vertices);
		memcpy(position_ptr, get_positions(), num_vertices * sizeof(vec3));

		dx_vertex_buffer_group_view vb = era_engine::animation::skinCloth(position_vertex_buffer, num_x, num_z);
		SubmeshInfo sm{};
		sm.baseVertex = 0;
		sm.firstIndex = 0;
		sm.numIndices = num_triangles * 3;
		sm.numVertices = num_vertices;

		dx_vertex_buffer_group_view prev = render_component->prevFrameVB;
		render_component->prevFrameVB = vb;

		if (!render_component->indexBuffer)
		{
			std::vector<indexed_triangle16> triangles;
			triangles.reserve(num_triangles);
			for (uint32 y = 0; y < num_z - 1; ++y)
			{
				for (uint32 x = 0; x < num_x - 1; ++x)
				{
					uint16 tlIndex = y * num_x + x;
					uint16 trIndex = tlIndex + 1;
					uint16 blIndex = tlIndex + num_x;
					uint16 brIndex = blIndex + 1;

					triangles.push_back({ tlIndex, blIndex, brIndex });
					triangles.push_back({ tlIndex, brIndex, trIndex });
				}
			}

			//render_component->indexBuffer = createIndexBuffer(sizeof(uint16), (uint32)triangles.size() * 3, triangles.data());
		}

		return { vb, prev, render_component->indexBuffer, sm };
	}
}