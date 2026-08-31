#include "game/camera/gameplay_tpp_camera_component.h"

#include <ecs/entity.h>

#include <rttr/registration>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<GameplayTppCameraComponent>("GameplayTppCameraComponent")
			.constructor<ref<Entity::EcsData>>();
	}

	GameplayTppCameraComponent::GameplayTppCameraComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{

	}

	

}