#include "ecs/base_components/name_component.h"

#include <rttr/registration>

namespace era_engine
{

	RTTR_REGISTRATION
	{
		using namespace rttr;
		rttr::registration::class_<NameComponent>("NameComponent")
			.constructor<ref<Entity::EcsData>, const char*>()
			.property("name", &NameComponent::name);
	}

	NameComponent::NameComponent(ref<Entity::EcsData> _data, const char* n) : Component(_data)
	{
		strncpy(name, n, sizeof(name));
		name[sizeof(name) - 1] = 0;
	}

	NameComponent::~NameComponent()
	{
	}

}
