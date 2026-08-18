#include "physics/aggregate_holder_component.h"
#include "physics/core/physics.h"

#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<AggregateHolderComponent>("AggregateHolderComponent")
			.constructor<>();
	}

	AggregateHolderComponent::AggregateHolderComponent()
	{
		ASSERT(!PhysicsEngine::get_physics_core()->is_gpu());
	}

	AggregateHolderComponent::AggregateHolderComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
		
	}

	AggregateHolderComponent::~AggregateHolderComponent()
	{

	}

}