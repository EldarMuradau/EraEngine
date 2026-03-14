#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include "physics/vehicles/base_vehicle_component.h"

#include "core/math.h"

#include "ecs/component.h"
#include "ecs/observable_member.h"

namespace era_engine::physics
{
	struct VehicleInputContext
	{
		bool accelerate_pressed = false;
		bool brake_pressed = false;
		bool reverse_pressed = false;

		bool steer_left_pressed = false;
		bool steer_right_pressed = false;

		// For analog steering
		float steer_amount = 0.0f;  // -1.0 to 1.0 (left to right)

		// For controller/analog input
		float throttle_axis = 0.0f; // 0 to 1
		float brake_axis = 0.0f; // 0 to 1
	};

	struct VehicleControlOutput
	{
		float brake = 0.0f;
		float throttle = 0.0f;
		float steer = 0.0f;
		bool reverse = false;
	};

	static constexpr float THROTTLE_RAMP_UP = 3.0f;
	static constexpr float THROTTLE_RAMP_DOWN = 5.0f;
	static constexpr float BRAKE_RAMP_UP = 4.0f;
	static constexpr float BRAKE_RAMP_DOWN = 6.0f;
	static constexpr float STEER_SPEED = 4.0f;
	static constexpr float STEER_RETURN_SPEED = 2.0f;

	class ERA_PHYSICS_API W4VehicleComponent : public VehicleBaseComponent
	{
	public:
		W4VehicleComponent() = default;
		W4VehicleComponent(ref<Entity::EcsData> _data);

		VehicleControlOutput process_input(const VehicleInputContext& input, float dt, float current_speed = 0.0f);

		float get_speed_based_steering_factor(bool is_accelerating, float current_speed);

		void process_reverse_request(const VehicleInputContext& input, float current_speed, VehicleControlOutput& output);
		void process_throttle_and_brake(const VehicleInputContext& input, float dt, float current_speed, VehicleControlOutput& output);
		void process_steering(const VehicleInputContext& input, float dt, float current_speed, VehicleControlOutput& output);
		void apply_vehicle_rules(const VehicleInputContext& input, float dt, float current_speed, VehicleControlOutput& output);

		float current_steer = 0.0f;
		float throttle = 0.0f;
		float brake = 0.0f;

		bool wants_reverse = false;

		ERA_VIRTUAL_REFLECT(VehicleBaseComponent)
	};
}