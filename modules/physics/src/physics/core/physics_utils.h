#pragma once

#include "physics_api.h"
#include "physics/physx_api.h"

#include <core/math.h>

#include <ecs/entity.h>

namespace era_engine::physics
{
	class BodyComponent;
	class DynamicBodyComponent;
	class CharacterControllerComponent;

	class ERA_PHYSICS_API PhysicsUtils final
	{
		PhysicsUtils() = delete;
	public:
		static physx::PxRigidDynamic* create_rigid_dynamic(const physx::PxTransform& transform, void* user_data);
		static physx::PxRigidStatic* create_rigid_static(const physx::PxTransform& transform, void* user_data);

		static BodyComponent* get_body_component(ref<Entity::EcsData> entity_data);
		static BodyComponent* get_body_component(weakref<Entity::EcsData> entity_data);
		static BodyComponent* get_body_component(Entity entity);

		static void manual_update_mass(DynamicBodyComponent* dynamic_body_component);

		static void manual_set_physics_transform_locked(Entity entity, const vec3& pos, const quat& rot, bool update_transform_component = true);
		static void manual_set_physics_transform_locked(Entity entity, const trs& transform, bool update_transform_component = true);

		static void manual_clear_force_and_torque(DynamicBodyComponent* body_component);

		static trs get_actor_world_pose_locked(Entity entity);

		static void update_mass_and_inertia(DynamicBodyComponent* body_component, float density);

		static void move_cct(CharacterControllerComponent* cct_component, const vec3& offset);
	};

	ERA_PHYSICS_API std::vector<physx::PxVec3> create_std_vector_px_vec3(const std::vector<vec3>& vec);

	ERA_PHYSICS_API std::vector<physx::PxVec2> create_std_vector_px_vec2(const std::vector<vec2>& vec);
}