#pragma once

#include <ecs/system.h>
#include <ecs/entity.h>

#include <core/math.h>

namespace era_engine
{
	class GameplayTppCameraSystem final : public System
	{
	public:
		GameplayTppCameraSystem(World* _world);
		~GameplayTppCameraSystem() override;

		void init() override;
		void update(float dt) override;

		ERA_VIRTUAL_REFLECT(System)
	};
}