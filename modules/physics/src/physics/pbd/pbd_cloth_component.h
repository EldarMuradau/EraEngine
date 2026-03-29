#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"

#include <ecs/component.h>

#include <core/math.h>

namespace era_engine::physics
{
	class ERA_PHYSICS_API PBDClothComponent : public Component
	{
	public:
		PBDClothComponent() = default;
		PBDClothComponent(ref<Entity::EcsData> _data);
	};
}