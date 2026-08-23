#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include "physics/collision_types.h"

#include "core/math.h"

#include "ecs/component.h"

namespace era_engine::physics
{
	class PhysicsMaterial;

	class ERA_PHYSICS_API PlanePhysicsComponent final : public Component
	{
	public:
		PlanePhysicsComponent() = default;
		PlanePhysicsComponent(ref<Entity::EcsData> _data, CollisionType _collision_type, const vec3& _point, const vec3& _norm = vec3(0.0f, 1.0f, 0.0f));
		~PlanePhysicsComponent() override;

		ref<PhysicsMaterial> material;

		ERA_VIRTUAL_REFLECT(Component)

	private:
		vec3 point = vec3::zero;
		vec3 normal = vec3::zero;
		physx::PxRigidStatic* plane = nullptr;
	};

	class ERA_PHYSICS_API TerrainPhysicsComponent final : public Component
	{
	public:
		TerrainPhysicsComponent() = default;
		TerrainPhysicsComponent(ref<Entity::EcsData> _data, CollisionType _collision_type, const vec3& _point);
		~TerrainPhysicsComponent() override;

		ref<PhysicsMaterial> material;

		ERA_VIRTUAL_REFLECT(Component)

	private:
		vec3 point = vec3::zero;
		physx::PxRigidStatic* terrain = nullptr;
		physx::PxShape* shape = nullptr;
	};

	ERA_PHYSICS_API physx::PxRigidDynamic* create_rigid_cube(physx::PxReal half_extent, const physx::PxVec3& position);

	ERA_PHYSICS_API void create_cube(physx::PxArray<physx::PxVec3>& tri_verts, physx::PxArray<physx::PxU32>& tri_indices, const physx::PxVec3& pos, physx::PxReal scaling);
}