#pragma once

#include "physics_api.h"
#include "physics/physx_api.h"

#include <core/math.h>

#include <ecs/component.h>

namespace era_engine::physics
{
	enum class PhysicalLimbStateType : uint8
	{
		KINEMATIC = 0,
		BLEND_OUT,
		SIMULATION,
		RAGDOLL,
		PARTIAL_DISMEMBERED,
		DISMEMBERED,

		USER_STATE
	};

	class ERA_PHYSICS_API BaseLimbState
	{
	public:
		BaseLimbState() = default;
		BaseLimbState(ComponentPtr _physical_animation_limb_component_ptr);
		virtual ~BaseLimbState();

		virtual PhysicalLimbStateType try_switch_to(PhysicalLimbStateType desired_type) const;

		virtual void on_enter();

		virtual void update(float dt);

		virtual void on_exit();

	protected:
		ComponentPtr physical_animation_limb_component_ptr = nullptr;
	};
}