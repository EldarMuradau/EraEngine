#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include "physics/vehicles/w4_vehicle_component.h"
#include "physics/vehicles/tank_vehicle_component.h"

#include <core/math.h>
#include <core/sync.h>

#include <ecs/system.h>
#include <ecs/base_components/transform_component.h>

#include <entt/entt.hpp>

namespace era_engine::physics
{
	class VehicleSystem : public System
	{
	public:
		VehicleSystem(World* _world);
		~VehicleSystem() override;

		void init() override;
		void update(float dt) override;

		void process_added_vehicles();

		void on_vehicle_created(entt::registry& registry, entt::entity entity_handle);
		void on_vehicle_released(entt::registry& registry, entt::entity entity_handle);

		ERA_VIRTUAL_REFLECT(System)

	private:
		SpinLock sync;
		std::vector<Entity::Handle> vehicles_to_init;

		entt::group<entt::owned_t<>, entt::get_t<TransformComponent,
			W4VehicleComponent>> w4_vehicles_group;

		entt::group<entt::owned_t<>, entt::get_t<TransformComponent,
			W4VehicleComponent>> tank_vehicles_group;
	};
}