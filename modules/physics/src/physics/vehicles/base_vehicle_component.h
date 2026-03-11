#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include "physics/snippets_utils/enginedrivetrain/EngineDrivetrain.h"
#include "physics/material.h"
#include "physics/collision_types.h"

#include "core/math.h"

#include "ecs/component.h"
#include "ecs/entity.h"

namespace era_engine::physics
{
	class ERA_PHYSICS_API VehicleBaseComponent : public Component
	{
	public:
		VehicleBaseComponent() = default;
		VehicleBaseComponent(ref<Entity::EcsData> _data);

		CollisionType collision_type = CollisionType::VEHICLE;

		ERA_VIRTUAL_REFLECT(Component)

	protected:
		virtual void init_material_friction_table();

	protected:
		ref<PhysicsMaterial> material;

		snippetvehicle2::EngineDriveVehicle* vehicle = nullptr;
		physx::vehicle2::PxVehiclePhysXSimulationContext* vehicle_simulation_context = nullptr;

		physx::vehicle2::PxVehiclePhysXMaterialFriction material_frictions[16];

		uint32 nb_material_frictions = 0;
		float default_material_friction = 1.0f;

		uint32 target_gear_command = physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR;

		friend class VehicleSystem;
	};

	class ERA_PHYSICS_API VehicleUtils
	{
	public:
		VehicleUtils() = delete;

		static VehicleBaseComponent* get_vehicle_component(Entity entity);
	};
}