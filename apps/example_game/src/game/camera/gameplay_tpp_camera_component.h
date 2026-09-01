#pragma once

#include <ecs/component.h>

#include <core/math.h>

namespace era_engine
{
	class GameplayTppCameraComponent final : public Component
	{
	public:
		GameplayTppCameraComponent() = default;
		GameplayTppCameraComponent(ref<Entity::EcsData> _data);

		struct OrbitState
		{
			float yaw = 0.0f; // Horizontal rotation
			float pitch = 20.0f; // -Vertical rotation
			float distance = 5.0f;
			float height = 2.0f;
		};

		float camera_sensitivity = 0.07f;
		float camera_min_pitch = -30.0f; // Allow looking up
		float camera_max_pitch = 60.0f; // Allow looking down
		float camera_smooth_speed = 25.0f;

		OrbitState orbit;

		ERA_VIRTUAL_REFLECT(Component)
	};
}