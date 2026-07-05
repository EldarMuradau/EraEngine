#include "physics/destructions/destructible_component.h"

#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<DestructibleComponent>("DestructibleComponent")
			.constructor<>();
	}

	DestructibleComponent::DestructibleComponent()
	{
	}

	DestructibleComponent::DestructibleComponent(ref<Entity::EcsData> _data, Type _base_type)
		: Component(_data)
		, base_type(_base_type)
	{
	}

	DestructibleComponent::~DestructibleComponent()
	{
	}

}