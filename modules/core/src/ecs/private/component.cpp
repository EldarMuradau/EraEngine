#include "ecs/component.h"
#include "ecs/world.h"

#include <rttr/registration>

namespace era_engine
{

	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<Component>("Component")
			.constructor<>()
			.constructor<ref<Entity::EcsData>>();
	}

	Component::Component(ref<Entity::EcsData> _data) noexcept
		: component_data(_data)
	{
	}

	Component::Component(const Component& _component) noexcept
		: component_data(_component.component_data)
	{
	}

	Component::Component(Component&& _component) noexcept
	{
		component_data = std::move(_component.component_data);
	}

	Component::~Component()
	{
	}

	Component& Component::operator=(const Component& _component)  noexcept
	{
		if (this != &_component)
		{
			component_data = _component.component_data;
		}
		return *this;
	}

	Component& Component::operator=(Component&& _component)  noexcept
	{
		if (this != &_component)
		{
			component_data = std::move(_component.component_data);
		}
		return *this;
	}

	World* Component::get_world() const
	{
		return component_data->world;
	}

	Entity Component::get_entity() const
	{
		return Entity(component_data);
	}

	Entity::Handle era_engine::Component::get_handle() const
	{
		return component_data->entity_handle;
	}

	ComponentDataPtr::ComponentDataPtr(const Component* _component)
		: component_weak_data(_component->component_data)
	{
	}

	ComponentPtr::ComponentPtr(Component* _component)
		: component(_component)
	{
	}

	const Component* ComponentPtr::get() const
	{
		return component;
	}

	Component* ComponentPtr::get_for_write()
	{
		return component;
	}

	bool ComponentPtr::is_empty() const
	{
		return component == nullptr;
	}

	Entity ComponentPtr::get_entity() const
	{
		if (component == nullptr)
		{
			return Entity::Null;
		}
		return component->get_entity();
	}

}