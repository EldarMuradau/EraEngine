#include "physics/cct_component.h"
#include "physics/core/physics.h"

#include "ecs/world.h"
#include "ecs/base_components/transform_component.h"

#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<CharacterControllerComponent>("CharacterControllerComponent")
			.constructor<>();
	}

	CharacterControllerComponent::CharacterControllerComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
		velocity.set_component(ComponentPtr{this});
		offset.set_component(ComponentPtr{ this });
		collision_type.set_component(ComponentPtr{ this });
		collision_filter_data.set_component(ComponentPtr{ this });
	}

	CharacterControllerComponent::~CharacterControllerComponent()
	{
	}
}