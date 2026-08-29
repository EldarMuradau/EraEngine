#pragma once

#include "ecs/system.h"

#include "motion_matching/array.h"

#include "core/math.h"

namespace era_engine
{
	class RendererHolderRootComponent;

	class MotionSystem final : public System
	{
	public:
		MotionSystem(World* _world);
		~MotionSystem() override;

		void init() override;
		void update(float dt) override;

		void update_base(float dt);
		void reset_input(float dt);

		ERA_VIRTUAL_REFLECT(System)

	private:
		RendererHolderRootComponent* renderer_holder_rc = nullptr;
	};
}