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

    VehicleControlOutput W4VehicleComponent::process_input(const VehicleInputContext& input, float dt, float current_speed)
    {
        VehicleControlOutput output;

        process_throttle_and_brake(input, dt, current_speed, output);
        process_steering(input, dt, current_speed, output);
        process_reverse_request(input, current_speed, output);
        apply_vehicle_rules(input, dt, current_speed, output);

        return output;
    }

    float W4VehicleComponent::get_speed_based_steering_factor(bool is_accelerating, float current_speed)
    {
        if (current_speed < 10.0f)
        {
            return 1.0f;
        }
        else if (current_speed < 30.0f)
        {
            return 0.8f;
        }
        else if (current_speed < 60.0f)
        {
            return 0.6f;
        }

        return 0.4f;
    }

    void W4VehicleComponent::process_reverse_request(const VehicleInputContext& input, float current_speed, VehicleControlOutput& output)
    {
        if (input.reverse_pressed)
        {
            if (std::abs(current_speed) < 1.0f)
            {
                wants_reverse = true;
            }
            else
            {
                wants_reverse = false;
                output.brake = std::max(output.brake, 0.5f);
            }
        }
        else
        {
            wants_reverse = false;
        }

        output.reverse = wants_reverse;
    }

    void W4VehicleComponent::process_throttle_and_brake(const VehicleInputContext& input, float dt, float current_speed, VehicleControlOutput& output)
    {
        if (input.throttle_axis > 0.0f || input.brake_axis > 0.0f)
        {
            throttle = input.throttle_axis;
            brake = input.brake_axis;
        }
        else
        {
            if (input.accelerate_pressed)
            {
                throttle = std::min(1.0f, throttle + dt * THROTTLE_RAMP_UP);
            }
            else
            {
                throttle = std::max(0.0f, throttle - dt * THROTTLE_RAMP_DOWN);
            }

            if (input.brake_pressed)
            {
                brake = std::min(1.0f, brake + dt * BRAKE_RAMP_UP);
            }
            else
            {
                brake = std::max(0.0f, brake - dt * BRAKE_RAMP_DOWN);
            }
        }

        output.throttle = throttle;
        output.brake = brake;
    }

    void W4VehicleComponent::process_steering(const VehicleInputContext& input, float dt, float current_speed, VehicleControlOutput& output)
    {
        float target_steer = 0.0f;

        if (std::abs(input.steer_amount) > 0.01f)
        {
            target_steer = input.steer_amount;
        }
        else
        {
            bool left_pressed = input.steer_left_pressed;
            bool right_pressed = input.steer_right_pressed;

            if (left_pressed && !right_pressed)
            {
                target_steer = 1.0f; // Full left
            }
            else if (right_pressed && !left_pressed)
            {
                target_steer = -1.0f; // Full right
            }
            else if (left_pressed && right_pressed)
            {
                target_steer = 0.0f;
            }
            else
            {
                target_steer = 0.0f;
            }
        }

        if (target_steer > current_steer)
        {
            current_steer = std::min(target_steer, current_steer + STEER_SPEED * dt);
        }
        else if (target_steer < current_steer)
        {
            current_steer = std::max(target_steer, current_steer - STEER_SPEED * dt);
        }

        float speed_factor = get_speed_based_steering_factor(output.throttle > 0.0f ? 1.0f : 0.0f, current_speed);

        output.steer = clamp(current_steer * speed_factor, -1.0f, 1.0f);
    }

    void W4VehicleComponent::apply_vehicle_rules(const VehicleInputContext& input, float dt, float current_speed, VehicleControlOutput& output)
	{
        if (wants_reverse && current_speed > 1.0f)
        {
            output.brake = std::max(output.brake, 0.8f);
            output.throttle = 0.0f;
        }
        else if (!wants_reverse && current_speed < -1.0f)
        {
            output.brake = std::max(output.brake, 0.8f);
            output.throttle = 0.0f;
        }

        if (!input.steer_left_pressed && 
            !input.steer_right_pressed && 
            std::abs(current_speed) > 0.5f)
        {
            if (current_steer >= 0.01f)
            {
                current_steer = std::max(0.0f, current_steer - STEER_RETURN_SPEED * dt);
            }
            else if (current_steer <= -0.01f)
            {
                current_steer = std::min(0.0f, current_steer + STEER_RETURN_SPEED * dt);
            }
        }
	}

}