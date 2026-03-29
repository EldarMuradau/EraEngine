#include "ecs/world.h"
#include "ecs/base_components/base_components.h"
#include "ecs/world_system_scheduler.h"

#include "core/sync.h"
#include "core/log.h"
#include "core/ecs/tags_component.h"

namespace era_engine
{
	std::unordered_map<std::string, World*> worlds;

	World::World(const std::string& _name)
	{
		world_data = new WorldData();
		world_data->name = _name;
		world_data->scheduler = new WorldSystemScheduler(this, 1, 1);

		world_data->scheduler->set_fixed_update_rate(60.0);

		worlds.emplace(_name, this);
	}

	World::~World()
	{
		destroy();
	}

	void World::init()
	{
		ref<Entity::EcsData> new_data = make_ref<Entity::EcsData>(world_data->registry.create(), Entity::NullHandle, this, &world_data->registry);
		world_data->entity_datas.emplace(new_data->entity_handle, new_data);
		world_data->root_entity = Entity(new_data);
		world_data->root_entity.add_component<TransformComponent>();
		world_data->root_entity.add_component<NameComponent>("RootEntity");
		world_data->root_entity.add_component<TagsComponent>();
	}

	Entity World::create_entity()
	{
		std::lock_guard _lock{ world_data->sync};

		ref<Entity::EcsData> new_data = make_ref<Entity::EcsData>(world_data->registry.create(), world_data->root_entity.get_handle(), this, &world_data->registry);
		world_data->entity_datas.emplace(new_data->entity_handle, new_data);

		Entity entity = Entity(new_data);
		add_base_components(entity);

		EntityContainer::emplace_pair(this, world_data->root_entity.get_handle(), new_data->entity_handle);

		return entity;
	}

	Entity World::create_entity(const char* _name)
	{
		Entity created_entity = create_entity();
		created_entity.add_component<NameComponent>(_name);
		return created_entity;
	}

	Entity World::create_entity(Entity::Handle _handle)
	{
		std::lock_guard _lock{ world_data->sync };

		if (world_data->entity_datas.find(_handle) == world_data->entity_datas.end())
		{
			if (world_data->registry.create(_handle) == Entity::NullHandle)
			{
				LOG_ERROR("ECS> Entity creation failed!");
			}

			ref<Entity::EcsData> new_data = make_ref<Entity::EcsData>(_handle, world_data->root_entity.get_handle(), this, &world_data->registry);
			world_data->entity_datas.emplace(new_data->entity_handle, new_data);

			Entity entity = Entity(new_data);
			add_base_components(entity);

			EntityContainer::emplace_pair(this, world_data->root_entity.get_handle(), new_data->entity_handle);

			return entity;
		}
		return Entity(world_data->entity_datas[_handle]);
	}

	Entity World::create_entity(Entity::Handle _handle, const char* _name)
	{
		Entity created_entity = create_entity(_handle);
		created_entity.add_component<NameComponent>(_name);
		return created_entity;
	}

	uint64 World::get_fixed_frame_id() const
	{
		return fixed_frame_id;
	}

	float World::get_fixed_update_dt() const
	{
		return fixed_update_dt;
	}

	void World::destroy_entity(const Entity& _entity)
	{
		destroy_entity(_entity.get_handle());
	}

	void World::destroy_entity(Entity::Handle _handle, bool _destroy_childs, bool _destroy_components)
	{
		if (_handle == Entity::NullHandle)
		{
			return;
		}

		if (_destroy_childs)
		{
			for (const Entity::Handle child : EntityContainer::get_childs(this, _handle))
			{
				destroy_entity(child);
			}
		}

		world_data->registry.destroy(_handle);
		world_data->entity_datas.erase(_handle);
	}

	Entity World::try_create_entity_in_place(const Entity& place, const char* _name)
	{
		if(place.is_valid())
		{
			return create_entity(place.internal_data->entity_handle, _name);
		}
		return Entity::Null;
	}

	Entity World::get_entity(Entity::Handle _handle)
	{
		if (world_data->entity_datas.find(_handle) == world_data->entity_datas.end())
		{
			return Entity();
		}
		return Entity(world_data->entity_datas[_handle]);
	}

	void World::destroy(bool _destroy_components)
	{
		for (auto iter = world_data->entity_datas.begin(); iter != world_data->entity_datas.end();)
		{
			if (iter->first == Entity::NullHandle)
			{
				continue;
			}

			world_data->registry.destroy(iter->first);
			iter = world_data->entity_datas.erase(iter);
			if (iter == world_data->entity_datas.end())
			{
				break;
			}
			++iter;
		}
		world_data->registry.clear();
		world_data->entity_datas.clear();

		delete world_data;
	}

	void World::add_tag(const std::string& tag)
	{
		world_data->scheduler->refresh_graph();
		world_data->root_entity.get_component<TagsComponent>()->add_tag(tag);
	}

	bool World::remove_tag(const std::string& tag)
	{
		world_data->scheduler->refresh_graph();
		return world_data->root_entity.get_component<TagsComponent>()->remove_tag(tag);
	}

	bool World::has_tag(const std::string& tag) const
	{
		return world_data->root_entity.get_component<TagsComponent>()->has_tag(tag);
	}

	void World::clear_tags()
	{
		world_data->root_entity.get_component<TagsComponent>()->clear();
	}

	const TagsContainer& World::get_tags_container() const
	{
		return world_data->root_entity.get_component<TagsComponent>()->get_container();
	}

	size_t World::size() const noexcept
	{
		return world_data->entity_datas.size();
	}

	entt::registry& World::get_registry()
	{
		return world_data->registry;
	}

	WorldSystemScheduler* World::get_system_scheduler() const
	{
		return world_data->scheduler;
	}

	Entity World::get_root_entity() const
	{
		return world_data->root_entity;
	}

	void World::add_base_components(Entity& entity)
	{
		entity.add_component<TransformComponent>();
	}

	World* get_world_by_name(const std::string& _name)
	{
		return worlds[_name];
	}

	std::unordered_map<std::string, World*>& get_worlds()
	{
		return worlds;
	}

}