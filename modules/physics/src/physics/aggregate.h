#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"

#include "ecs/reflection.h"

namespace era_engine::physics
{
	class ERA_PHYSICS_API Aggregate final
	{
	public:
		Aggregate() = default;
		Aggregate(uint8 _nb_actors, bool _self_collisions = true);
		~Aggregate();

		void add_actor(physx::PxActor* actor);
		void remove_actor(physx::PxActor* actor);

		uint8 get_nb_actors() const;
		bool is_self_collision() const;

		physx::PxAggregate* get_native_aggregate();

		ERA_REFLECT

	private:
		physx::PxAggregate* aggregate = nullptr;

		uint8 nb_actors = 0;
		bool self_collisions = true;
	};
}