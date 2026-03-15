#include "physics/vehicles/base_vehicle_component.h"
#include "physics/core/physics.h"
#include "physics/vehicles/tank_vehicle_component.h"
#include "physics/vehicles/w4_vehicle_component.h"

#include <rttr/registration>

namespace era_engine::physics
{

	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<VehicleBaseComponent>("VehicleBaseComponent")
			.constructor<>();
	}

	VehicleBaseComponent::VehicleBaseComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
		init_material_friction_table();
	}

	void VehicleBaseComponent::init_material_friction_table()
	{
		material = PhysicsEngine::get_physics_core()->create_material(0.8f, 0.8f, 0.6f);

		material_frictions[0].friction = 1.0f;
		material_frictions[0].material = material->get_native_material();
		default_material_friction = 1.0f;
		nb_material_frictions = 1;
	}

	VehicleBaseComponent* VehicleUtils::get_vehicle_component(Entity entity)
	{
		if (W4VehicleComponent* w4_vehicle_component = entity.get_component_if_exists<W4VehicleComponent>())
		{
			return w4_vehicle_component;
		}
		else if (TankVehicleComponent* tank_vehicle_component = entity.get_component_if_exists<TankVehicleComponent>())
		{
			return tank_vehicle_component;
		}

		return nullptr;
	}
}