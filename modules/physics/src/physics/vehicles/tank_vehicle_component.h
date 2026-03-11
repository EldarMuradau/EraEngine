#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include "physics/vehicles/base_vehicle_component.h"

#include "core/math.h"

#include "ecs/component.h"
#include "ecs/observable_member.h"

namespace era_engine::physics
{
	class ERA_PHYSICS_API TankVehicleComponent : public VehicleBaseComponent
	{
	public:
		TankVehicleComponent() = default;
		TankVehicleComponent(ref<Entity::EcsData> _data);

		ERA_VIRTUAL_REFLECT(VehicleBaseComponent)
	};
}