#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include "physics/basic_objects.h"
#include "physics/aggregate.h"

#include <ecs/component.h>

namespace era_engine::physics
{
	class ERA_PHYSICS_API AggregateHolderComponent : public Component
	{
	public:
		AggregateHolderComponent();

		AggregateHolderComponent(ref<Entity::EcsData> _data);
		~AggregateHolderComponent() override;

		bool enable_self_collision = true;
		uint32 max_actors = 32;

		ERA_VIRTUAL_REFLECT(Component)

	private:
		ref<Aggregate> aggregate;

		friend class Physics;
		friend class PhysicsSystem;
	};
}