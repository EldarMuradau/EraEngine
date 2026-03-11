#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include "physics/vehicles/base_vehicle_component.h"

#include "core/math.h"

#include "ecs/component.h"
#include "ecs/observable_member.h"

namespace era_engine::physics
{
	class ERA_PHYSICS_API W4VehicleComponent : public VehicleBaseComponent
	{
	public:
		W4VehicleComponent() = default;
		W4VehicleComponent(ref<Entity::EcsData> _data);

		ERA_VIRTUAL_REFLECT(VehicleBaseComponent)
	};
}