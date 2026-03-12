#include "physics/aggregate.h"
#include "physics/core/physics.h"

namespace era_engine::physics
{
	Aggregate::Aggregate(uint8 _nb_actors, bool _self_collisions)
		: nb_actors(_nb_actors), self_collisions(_self_collisions)
	{
		using namespace physx;

		auto& physics_ref = PhysicsHolder::physics_ref;

		aggregate = physics_ref->get_physics()->createAggregate(nb_actors, nb_actors, PxGetAggregateFilterHint(PxAggregateType::eGENERIC, self_collisions));
		physics_ref->get_scene()->addAggregate(*aggregate);
	}

	Aggregate::~Aggregate()
	{
		PX_RELEASE(aggregate)
	}

	void Aggregate::add_actor(physx::PxActor* actor)
	{
		aggregate->addActor(*actor);
	}

	void Aggregate::remove_actor(physx::PxActor* actor)
	{
		aggregate->removeActor(*actor);
	}

	uint8 Aggregate::get_nb_actors() const
	{
		return nb_actors;
	}

	bool Aggregate::is_self_collision() const
	{
		return self_collisions;
	}

	physx::PxAggregate* Aggregate::get_native_aggregate()
	{
		return aggregate;
	}
}