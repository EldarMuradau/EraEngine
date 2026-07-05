#include "motion_matching/trajectory/trajectory_component.h"

#include <rttr/registration>

namespace era_engine
{

	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<TrajectoryComponent>("TrajectoryComponent")
			.constructor<ref<Entity::EcsData>>();
	}

	TrajectoryComponent::TrajectoryComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
		trajectory_desired_velocities.resize(number_of_trajectories);
		trajectory_desired_rotations.resize(number_of_trajectories);
		trajectory_positions.resize(number_of_trajectories);
		trajectory_rotations.resize(number_of_trajectories);
		trajectory_velocities.resize(number_of_trajectories);
		trajectory_accelerations.resize(number_of_trajectories);
		trajectory_angular_velocities.resize(number_of_trajectories);

		time_offsets.resize(number_of_trajectories);
	}

	TrajectoryComponent::~TrajectoryComponent()
	{
	}

}