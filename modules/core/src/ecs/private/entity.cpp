#include "ecs/entity.h"
#include "ecs/world.h"
#include "ecs/component.h"
#include "ecs/base_components/transform_component.h"

#include "core/sync.h"

#include <rttr/registration>

#include <algorithm>

namespace era_engine
{

    Entity Entity::null = Entity(nullptr);

    RTTR_REGISTRATION
    {
        using namespace rttr;

        registration::class_<Entity>("Entity")
            .constructor<>()
            .constructor<ref<Entity::EcsData>>();
    }

    Entity::Entity(const Entity& _entity)
        : internal_data(_entity.internal_data)
    {
    }

    Entity::Entity(Entity&& _entity)
    {
        internal_data = std::move(_entity.internal_data);
    }


    Entity::Entity(ref<EcsData> _data)
        : internal_data(_data)
    {
    }

    Entity::~Entity()
    {
    }

    Entity& Entity::operator=(const Entity& _entity)
    {
        internal_data = _entity.internal_data;
        return *this;
    }

    Entity& Entity::operator=(Entity&& _entity)
    {
        internal_data = std::move(_entity.internal_data);
        return *this;
    }

    bool Entity::operator==(const Entity& _other) const
    {
        return internal_data == _other.internal_data;
    }

    bool Entity::operator!=(const Entity& _other) const
    {
        return !(*this == _other);
    }

    bool Entity::operator==(Entity::Handle _handle) const
    {
        return internal_data->entity_handle == _handle;
    }

    bool Entity::is_valid() const
    {
        return internal_data != nullptr &&
               internal_data->entity_handle != Entity::null_handle &&
               internal_data->world != nullptr &&
               internal_data->world->is_entity_valid(internal_data->entity_handle);
    }

    weakref<Entity::EcsData> Entity::get_data_weakref() const
    {
        return weakref<EcsData>(internal_data);
    }

    void Entity::set_parent(Entity::Handle parent_handle)
    {
        EntityContainer::erase_pair(get_world(), internal_data->parent_handle, internal_data->entity_handle);
        EntityContainer::emplace_pair(get_world(), parent_handle, internal_data->entity_handle);
        internal_data->parent_handle = parent_handle;
    }

    World* Entity::get_world() const
    {
        return internal_data->world;
    }

    Entity::Handle Entity::get_handle() const
    {
        return internal_data->entity_handle;
    }

    Entity::Handle Entity::get_parent_handle() const
    {
        return internal_data->parent_handle;
    }

    void EntityContainer::emplace_pair(World* world, Entity::Handle parent, Entity::Handle child)
    {
        std::lock_guard _lock(world->world_data->hierarchy_sync);
        world->world_data->hierarchy[parent].childs.push_back(child);
    }

    void EntityContainer::erase(World* world, Entity::Handle parent)
    {
        std::vector<Entity::Handle> childs;
        {
            std::lock_guard _lock(world->world_data->hierarchy_sync);

            auto& hierarchy = world->world_data->hierarchy;
            auto node_iter = hierarchy.find(parent);
            if (node_iter == hierarchy.end())
            {
                return;
            }

            childs = std::move(node_iter->second.childs);
            hierarchy.erase(node_iter);
        }

        const Entity::Handle root_handle = world->get_root_entity().get_handle();
        for (const Entity::Handle child_handle : childs)
        {
            Entity child = world->get_entity(child_handle);
            if (child.is_valid())
            {
                child.set_parent(root_handle);
            }
        }
    }

    void EntityContainer::erase_pair(World* world, Entity::Handle parent, Entity::Handle child)
    {
        std::lock_guard _lock(world->world_data->hierarchy_sync);

        auto& hierarchy = world->world_data->hierarchy;
        auto node_iter = hierarchy.find(parent);
        if (node_iter == hierarchy.end())
        {
            return;
        }

        std::vector<Entity::Handle>& childs = node_iter->second.childs;
        auto child_iter = std::find(childs.begin(), childs.end(), child);
        if (child_iter != childs.end())
        {
            childs.erase(child_iter);
        }

        if (childs.empty())
        {
            hierarchy.erase(node_iter);
        }
    }

    std::vector<Entity::Handle> EntityContainer::get_childs(World* world, Entity::Handle parent)
    {
        std::lock_guard _lock(world->world_data->hierarchy_sync);

        const auto& hierarchy = world->world_data->hierarchy;
        const auto node_iter = hierarchy.find(parent);
        if (node_iter == hierarchy.end())
        {
            return std::vector<Entity::Handle>();
        }

        return node_iter->second.childs;
    }

    std::vector<Entity> EntityContainer::get_entity_childs(World* world, Entity::Handle parent)
    {
        const std::vector<Entity::Handle> childs = get_childs(world, parent);

        std::vector<Entity> result;
        result.reserve(childs.size());

        for (const Entity::Handle handle : childs)
        {
            result.emplace_back(world->get_entity(handle));
        }

        return result;
    }

    EntityPtr::EntityPtr(Entity entity)
        : data(entity.get_data_weakref())
    {
    }
    EntityPtr::EntityPtr(const weakref<Entity::EcsData>& entity_data)
        : data(entity_data)
    {
    }

    Entity EntityPtr::get() const
    {
        if (std::shared_ptr<Entity::EcsData> internal_data = data.lock())
        {
            return Entity(internal_data);
        }
        return Entity::null;
    }

    bool EntityPtr::operator==(const EntityPtr& _other) const
    {
        return get() == _other.get();
    }

    bool EntityPtr::operator!=(const EntityPtr& _other) const
    {
        return !(*this == _other);
    }

    const weakref<Entity::EcsData>& EntityPtr::get_data() const
    {
        return data;
    }

    bool EntityPtr::is_empty() const
    {
        return data.lock() == nullptr;
    }

}
