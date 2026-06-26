#pragma once

#include "physics_api.h"
#include "physics/physx_api.h"

#include <core/math.h>

#include <ecs/component.h>

namespace era_engine::physics
{
	class PhysicalAnimationLimbComponent;

	enum class SimulationStateType : uint8
	{
		DISABLED = 0,
		BLEND_IN,
		ENABLED,
		BLEND_OUT,
		RAGDOLL,

		USER_STATE
	};

	class ERA_PHYSICS_API BaseSimulationState
	{
	public:
		BaseSimulationState() = default;
		BaseSimulationState(ComponentPtr _physical_animation_component_ptr);
		virtual ~BaseSimulationState();

		virtual SimulationStateType try_switch_to(SimulationStateType desired_type) const;

		virtual void on_enter();
		virtual void on_exit();

		virtual void update(float dt);

	protected:
		void traverse_simulation_graph(const std::function<void(PhysicalAnimationLimbComponent*)>& action_for_limb);

	protected:
		ComponentPtr physical_animation_component_ptr = nullptr;
	};
}