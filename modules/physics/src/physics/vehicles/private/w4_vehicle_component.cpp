#include "physics/vehicles/w4_vehicle_component.h"

#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<W4VehicleComponent>("W4VehicleComponent")
			.constructor<>();
	}

	W4VehicleComponent::W4VehicleComponent(ref<Entity::EcsData> _data)
		: VehicleBaseComponent(_data)
	{
	}

}