#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include "physics/basic_objects.h"

#include <ecs/component.h>

namespace era_engine::physics
{
	//ERA_PHYSICS_API void connect_cube_to_soft_body(physx::PxRigidDynamic* cube, physx::PxReal cube_half_extent, const physx::PxVec3& cube_position, physx::PxSoftBody* soft_body, physx::PxU32 point_grid_resolution = 10);

	class ERA_PHYSICS_API SoftBodyComponent : public Component
	{
	public:
		SoftBodyComponent() = default;

		SoftBodyComponent(ref<Entity::EcsData> _data);
		~SoftBodyComponent() override;

		uint32 get_nb_vertices() const;

		ERA_VIRTUAL_REFLECT(Component)

	private:
		void create_soft_body(const physx::PxCookingParams& params, 
							  const physx::PxArray<physx::PxVec3>& tri_verts,
							  const physx::PxArray<physx::PxU32>& tri_indices,
							  bool use_collision_mesh_for_simulation = false);

		void copy_deformed_vertices_from_gpu_async(CUstream stream);

		void copy_deformed_vertices_from_gpu();

		physx::PxVec4* positions_inv_mass = nullptr;
		//physx::PxSoftBody* soft_body = nullptr;

		friend class Physics;
	};
}