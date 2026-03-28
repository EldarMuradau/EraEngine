#pragma once

#include "physics_api.h"
#include "physics/physx_api.h"

#include "physics/physical_animation/limb_states/base_limb_state.h"

namespace era_engine::physics
{
    class ERA_PHYSICS_API SimulationLimbState : public BaseLimbState
    {
    public:
        SimulationLimbState(ComponentPtr _limb_component_ptr);

        void update(float dt) override;

        void on_enter() override;

        PhysicalLimbStateType try_switch_to(PhysicalLimbStateType desired_state) const override;
    };
}