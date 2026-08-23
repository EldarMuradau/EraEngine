#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"

#include <core/math.h>

#include <ecs/entity.h>

namespace era_engine
{
	struct MeshAsset;
	struct SubmeshAsset;
}

namespace era_engine::physics
{
	class ShapeComponent;

	class ERA_PHYSICS_API ShapeUtils final
	{
		ShapeUtils() = delete;
	public:
		static physx::PxConvexMesh* build_convex_mesh(const MeshAsset* asset, const vec3& size);
		static physx::PxTriangleMesh* build_triangle_mesh(const MeshAsset* asset);

		static bool is_trigger(const physx::PxFilterData& data);

		static ShapeComponent* get_shape_component(Entity entity);

		static void enable_shape_visualization(physx::PxShape* shape, const bool enable);
		static void enable_shape_in_contact_tests(physx::PxShape* shape, const bool enable);
		static void enable_shape_in_scene_query_tests(physx::PxShape* shape, const bool enable);

		static void setup_filtering(World* world, 
			physx::PxShape* shape, 
			uint32 collision_type, 
			std::optional<uint32> collision_filter_data);

		static float signed_volume_of_triangle(const vec3& p1, const vec3& p2, const vec3& p3);

		static float volume_of_mesh(const SubmeshAsset& mesh);
	};

}