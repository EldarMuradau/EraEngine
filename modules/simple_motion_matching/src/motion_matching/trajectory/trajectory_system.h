#pragma once

#include "ecs/system.h"

#include "motion_matching/array.h"

#include "core/math.h"

namespace era_engine
{
	class RendererHolderRootComponent;

	class TrajectoryMotionSystem final : public System
	{
	public:
		TrajectoryMotionSystem(World* _world);
		~TrajectoryMotionSystem() override;

		void init() override;
		void update(float dt) override;

		void debug_draw_update(float dt);

		ERA_VIRTUAL_REFLECT(System)

	private:
		void draw_trajectory(
			const slice1d<vec3>& trajectory_positions,
			const slice1d<quat>& trajectory_rotations,
			const slice1d<vec3>& trajectory_velocities,
			const vec4& color);

	private:
		RendererHolderRootComponent* renderer_holder_rc = nullptr;
	};
}