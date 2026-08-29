#pragma once

#include "motion_matching_api.h"

#include "motion_matching/array.h"

#include <core/math.h>

#include <ecs/component.h>

namespace era_engine
{
	class ERA_MOTION_MATCHING_API TrajectoryComponent final : public Component
	{
	public:
		TrajectoryComponent() = default;
		TrajectoryComponent(ref<Entity::EcsData> _data);

		~TrajectoryComponent() override;

		ERA_VIRTUAL_REFLECT(Component)

	public:
		array1d<vec3> trajectory_desired_velocities;
		array1d<quat> trajectory_desired_rotations;

		array1d<vec3> trajectory_positions;
		array1d<quat> trajectory_rotations;

		array1d<vec3> trajectory_velocities;
		array1d<vec3> trajectory_accelerations;

		array1d<vec3> trajectory_angular_velocities;

		size_t number_of_trajectories = 5;

		array1d<float> time_offsets;
	};
}