#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include "physics/collision_types.h"
#include "physics/material.h"

#include <ecs/component.h>
#include <ecs/observable_member.h>

namespace era_engine
{
	struct MeshAsset;
}

namespace era_engine::physics
{
	static constexpr size_t MAX_SHAPES_COUNT_PER_BODY = 16;

	template<typename T_>
	using IsShapeType = std::enable_if_t<std::is_base_of_v<class ShapeComponent, T_>, bool>;

	class ERA_PHYSICS_API ShapeComponent : public Component
	{
	public:
		ShapeComponent() = default;
		ShapeComponent(ref<Entity::EcsData> _data);
		~ShapeComponent() override;

		template<class T>
		T* is()
		{
			if constexpr (std::is_base_of_v<ShapeComponent, T>)
			{
				return static_cast<T*>(this);
			}
			return nullptr;
		}

		physx::PxShape* get_shape() const;
		const physx::PxGeometry& get_geometry() const;

		ObservableMember<vec3> local_position = vec3::zero;
		ObservableMember<quat> local_rotation = quat::identity;

		ObservableMember<CollisionType> collision_type = CollisionType::NONE;
		ObservableMember<std::optional<uint32>> collision_filter_data = std::optional<uint32>{};

		ObservableMember<bool> use_in_scene_queries = true;
		ObservableMember<bool> is_trigger = false;

		ref<PhysicsMaterial> material = nullptr;

		void sync_with_joint(weakref<Entity::EcsData> _entity_reference, uint32 _connected_joint_id);
		void set_attacment_state(bool active);

		ERA_VIRTUAL_REFLECT(Component)

	protected:
		void register_shape();

		virtual physx::PxShape* create_shape();

	protected:
		physx::PxShape* shape = nullptr;

		uint32 connected_joint_id;
		weakref<Entity::EcsData> entity_reference;
		bool attachment_active = false;

		friend class DynamicBodyComponent;
		friend class StaticBodyComponent;
		friend class PhysicsSystem;
		friend class ShapeSystem;
		friend class Physics;
	};

	class ERA_PHYSICS_API BoxShapeComponent final : public ShapeComponent
	{
	public:
		BoxShapeComponent() = default;
		BoxShapeComponent(ref<Entity::EcsData> _data);
		~BoxShapeComponent() override;

		vec3 half_extents = vec3(1.0f);

		ERA_VIRTUAL_REFLECT(ShapeComponent)

	protected:
		physx::PxShape* create_shape() override;
	};

	class ERA_PHYSICS_API SphereShapeComponent final : public ShapeComponent
	{
	public:
		SphereShapeComponent() = default;
		SphereShapeComponent(ref<Entity::EcsData> _data);
		~SphereShapeComponent() override;

		float radius = 1.0f;

		ERA_VIRTUAL_REFLECT(ShapeComponent)

	protected:
		physx::PxShape* create_shape() override;
	};

	class ERA_PHYSICS_API CapsuleShapeComponent final : public ShapeComponent
	{
	public:
		CapsuleShapeComponent() = default;
		CapsuleShapeComponent(ref<Entity::EcsData> _data);
		~CapsuleShapeComponent() override;

		float radius = 1.0f;
		float half_height = 1.0f;

		ERA_VIRTUAL_REFLECT(ShapeComponent)

	protected:
		physx::PxShape* create_shape() override;
	};

	class ERA_PHYSICS_API TriangleMeshShapeComponent final : public ShapeComponent
	{
	public:
		TriangleMeshShapeComponent() = default;
		TriangleMeshShapeComponent(ref<Entity::EcsData> _data);
		~TriangleMeshShapeComponent() override;

		vec3 size = vec3(1.0f);
		ref<MeshAsset> asset = nullptr;

		ERA_VIRTUAL_REFLECT(ShapeComponent)

	protected:
		physx::PxShape* create_shape() override;
	};

	class ERA_PHYSICS_API ConvexMeshShapeComponent final : public ShapeComponent
	{
	public:
		ConvexMeshShapeComponent() = default;
		ConvexMeshShapeComponent(ref<Entity::EcsData> _data);
		~ConvexMeshShapeComponent() override;

		vec3 size = vec3(1.0f);
		ref<MeshAsset> asset = nullptr;

		ERA_VIRTUAL_REFLECT(ShapeComponent)

	protected:
		physx::PxShape* create_shape() override;
	};
}