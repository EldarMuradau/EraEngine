#pragma once

#include "core_api.h"

#include "core/tags_container.h"

#include "ecs/entity.h"
#include "ecs/entity_utils.h"

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

#include <unordered_map>

namespace era_engine
{
	template <typename... Component_>
	struct ComponentsGroup {};

	template<typename... Type_>
	inline constexpr ComponentsGroup<Type_...> components_group{};

	class WorldSystemScheduler;

	class ERA_CORE_API World final
	{
		struct WorldData
		{
			std::mutex sync;

			std::unordered_map<Entity::Handle, ref<Entity::EcsData>> entity_datas;

			entt::registry registry;

			Entity root_entity;

			WorldSystemScheduler* scheduler = nullptr;

			std::string name;
		};

	public:
		static inline std::string GAMEPLAY_WORLD_NAME = std::string("GameplayWorld");

		World(const std::string& _name);
		~World();

		World(const World& _world) = delete;
		World& operator=(const World* _world) const = delete;

		void init();

		Entity create_entity();
		Entity create_entity(const char* _name);
		Entity create_entity(Entity::Handle _handle);
		Entity create_entity(Entity::Handle _handle, const char* _name);

		uint64 get_fixed_frame_id() const;
		float get_fixed_update_dt() const;

		void destroy_entity(const Entity& _entity);
		void destroy_entity(Entity::Handle _handle, bool _destroy_childs = true, bool _destroy_components = true);

		Entity try_create_entity_in_place(const Entity& place, const char* _name);

		Entity get_entity(Entity::Handle _handle) const;

		Entity get_root_entity() const;

		void destroy(bool _destroy_components = true);

		void add_tag(const std::string& tag);
		bool remove_tag(const std::string& tag);
		bool has_tag(const std::string& tag) const;
		void clear_tags();

		const TagsContainer& get_tags_container() const;

		size_t size() const noexcept;

		entt::registry& get_registry();

		WorldSystemScheduler* get_system_scheduler() const;

		template <typename Component_>
		Component_* get_root_component()
		{
			return world_data->root_entity.get_component_if_exists<Component_>();
		}

		template <typename Component_, typename... Args_>
		Component_* add_root_component(Args_&&... args)
		{
			if (!world_data->root_entity.has_component<Component_>())
			{
				return world_data->root_entity.add_component<Component_>(std::forward<Args_>(args)...);
			}

			return world_data->root_entity.get_component_if_exists<Component_>();
		}

		template <typename Component_>
		void copy_component_if_exists(Entity& src, Entity& dst)
		{
			if (Component_* comp = src.get_component_if_exists<Component_>())
			{
				dst.add_component<Component_>(*comp);
			}
		}

		template <typename FirstComponent_, typename... TailComponent_>
		void copy_components_if_exists(Entity& src, Entity& dst)
		{
			copy_component_if_exists<FirstComponent_>(src, dst);
			if constexpr (sizeof...(TailComponent_) > 0)
			{
				copy_components_if_exists<TailComponent_...>(src, dst);
			}
		}

		template <typename... Component_>
		auto view()
		{
			return world_data->registry.view<Component_...>();
		}

		template<typename... OwnedComponent_, typename... NonOwnedComponent_, typename... ExcludedComponents>
		auto group(ComponentsGroup<NonOwnedComponent_...> = {}, ComponentsGroup<ExcludedComponents...> = {})
		{
			return world_data->registry.group<OwnedComponent_...>(entt::get<NonOwnedComponent_...>, entt::exclude<ExcludedComponents...>);
		}

		template <typename Component_>
		uint32 number_of_components_of_type()
		{
			auto v = view<Component_>();
			return (uint32)v.size();
		}

		template <typename Context_, typename... Args_>
		Context_& create_or_get_context_variable(Args_&&... a)
		{
			return EntityUtils::create_or_get_context_variable<Context_, Args_...>(world_data->registry, std::forward<Args_>(a)...);
		}

		template <typename Context_>
		Context_& get_context_variable()
		{
			return EntityUtils::get_context_variable<Context_>(world_data->registry);
		}

		template <typename Context_>
		bool does_context_variable_exist()
		{
			return EntityUtils::does_context_variable_exist<Context_>(world_data->registry);
		}

		template <typename Context_>
		void delete_context_variable()
		{
			return EntityUtils::delete_context_variable<Context_>(world_data->registry);
		}

	private:
		template <typename Component_>
		void copy_component_pool_to(World& target)
		{
			auto v = view<Component_>();
			auto& s = world_data->registry.storage<Component_>();
			target.world_data->registry.insert<Component_>(v.begin(), v.end(), s.cbegin());
		}

		template <typename... Component_>
		void copy_component_pools_to(World& target)
		{
			(copy_component_pool_to<Component_>(target), ...);
		}

		template <typename... Component_>
		void copy_component_pools_to(ComponentsGroup<Component_...>, World& target)
		{
			(copy_component_pool_to<Component_>(target), ...);
		}

		void add_base_components(Entity& entity);

	public:
		ERA_REFLECT

	private:
		WorldData* world_data = nullptr;

		uint64 fixed_frame_id = 0;

		float fixed_update_dt = 0.0f;

		friend class Entity;
		friend class WorldSystemScheduler;
	};

	ERA_CORE_API World* get_world_by_name(const std::string& _name);

	ERA_CORE_API std::unordered_map<std::string, World*>& get_worlds();
}