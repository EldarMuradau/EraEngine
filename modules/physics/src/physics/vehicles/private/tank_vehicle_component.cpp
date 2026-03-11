#include "physics/vehicles/tank_vehicle_component.h"

#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<TankVehicleComponent>("TankVehicleComponent")
			.constructor<>();
	}

	TankVehicleComponent::TankVehicleComponent(ref<Entity::EcsData> _data)
		: VehicleBaseComponent(_data)
	{
	}

}