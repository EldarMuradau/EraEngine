#pragma once

#include "core_api.h"

#include "ecs/reflection.h"

#ifndef ECS_VALIDATE
#define ECS_VALIDATE 1
#endif // !ECS_VALIDATE

#ifndef ENTT_ASSERT
#if ECS_VALIDATE
#define ENTT_ASSERT(condition, ...) ASSERT(condition)
#else
#define ENTT_ASSERT(condition, ...)
#endif
#endif // !ENTT_ASSERT

#include <entt/entity/entity.hpp>
#include <entt/entity/registry.hpp>
#include <entt/entity/helper.hpp>

namespace era_engine
{
	class World;

	class ERA_CORE_API Entity final
	{
	public:
		using Handle = entt::entity;
		static inline const entt::null_t NullHandle = entt::null;

		static Entity Null;

		struct EcsData final
		{
			Entity::Handle entity_handle = Entity::NullHandle;
			Entity::Handle parent_handle = Entity::NullHandle;
			World* world = nullptr;
			entt::registry* native_registry = nullptr;
		};

		ERA_REFLECT

	public:
		Entity() = default;
		Entity(const Entity& _entity);
		Entity(Entity&& _entity);
		Entity(ref<EcsData> _data);
		~Entity();

		Entity& operator=(const Entity& _entity);
		Entity& operator=(Entity&& _entity);

		bool operator==(const Entity& _other) const;
		bool operator!=(const Entity& _other) const;
		bool operator==(Entity::Handle _handle) const;

		bool is_valid() const;

		World* get_world() const;
		Entity::Handle get_handle() const;
		Entity::Handle get_parent_handle() const;

		template <typename Component_, typename... Args_>
		Component_* add_component(Args_&&... a)
		{
			if (!has_component<Component_>())
			{
				return &internal_data->native_registry->emplace_or_replace<Component_>(internal_data->entity_handle, internal_data, std::forward<Args_>(a)...);
			}
			return get_component<Component_>();
		}

		template <typename Component_>
		uint32 get_component_index() const
		{
			auto& s = internal_data->native_registry->storage<Component_>();
			return (uint32)s.index(internal_data->entity_handle);
		}

		template <typename Component_>
		void remove_component()
		{
			internal_data->native_registry->remove<Component_>(internal_data->entity_handle);
		}

		template <typename Component_>
		bool has_component() const
		{
			return internal_data->native_registry->any_of<Component_>(internal_data->entity_handle);
		}

		template <typename Component_>
		Component_* get_component()
		{
			return &internal_data->native_registry->get<Component_>(internal_data->entity_handle);
		}

		template <typename Component_>
		const Component_* get_component() const
		{
			return &internal_data->native_registry->get<Component_>(internal_data->entity_handle);
		}

		template <typename Component_>
		Component_* get_component_if_exists()
		{
			return internal_data->native_registry->try_get<Component_>(internal_data->entity_handle);
		}

		template <typename Component_>
		const Component_* get_component_if_exists() const
		{
			return internal_data->native_registry->try_get<Component_>(internal_data->entity_handle);
		}

		weakref<Entity::EcsData> get_data_weakref() const;

		void set_parent(Entity::Handle parent_handle);

	private:
		ref<EcsData> internal_data = nullptr;

		friend class World;
		friend struct eeditor;
	};

	class ERA_CORE_API EntityPtr
	{
	public:
		EntityPtr() = default;
		EntityPtr(Entity entity);
		EntityPtr(const weakref<Entity::EcsData>& entity_data);

		bool operator==(const EntityPtr& _other) const;
		bool operator!=(const EntityPtr& _other) const;

		Entity get() const;

		const weakref<Entity::EcsData>& get_data() const;

		bool is_empty() const;

	private:
		weakref<Entity::EcsData> data;
	};

	struct ERA_CORE_API EntityNode
	{
		EntityNode() = default;
		~EntityNode() = default;

		std::vector<Entity::Handle> childs;
	};

	class ERA_CORE_API EntityContainer final
	{
		EntityContainer() = delete;

	public:
		static void emplace_pair(World* world, Entity::Handle parent, Entity::Handle child);

		static void erase(World* world, Entity::Handle parent);

		static void erase_pair(World* world, Entity::Handle parent, Entity::Handle child);

		static std::vector<Entity::Handle> get_childs(World* world, Entity::Handle parent);

		static std::vector<Entity> get_entity_childs(World* world, Entity::Handle parent);

	private:
		static inline std::unordered_map<Entity::Handle, EntityNode> container;

		friend class World;
	};
}