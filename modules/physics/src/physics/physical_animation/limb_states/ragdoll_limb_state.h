#pragma once

#include "physics_api.h"
#include "physics/physx_api.h"

#include "physics/physical_animation/limb_states/base_limb_state.h"

namespace era_engine::physics
{
    class ERA_PHYSICS_API RagdollLimbState final : public BaseLimbState
    {
    public:
        RagdollLimbState(ComponentPtr _limb_component_ptr);

        void on_enter() override;
    };
}