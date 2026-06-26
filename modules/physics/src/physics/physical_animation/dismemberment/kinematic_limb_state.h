#pragma once

#include "physics_api.h"

#include "physics/physical_animation/limb_states/base_limb_state.h"

namespace era_engine::physics
{
    class ERA_PHYSICS_API DismemberedLimbState final : public BaseLimbState
    {
    public:
        DismemberedLimbState(ComponentPtr _limb_component_ptr);

        void update(float dt) override;

        void on_enter() override;
        void on_exit() override;
    };

    class ERA_PHYSICS_API PartialDismemberedLimbState final : public BaseLimbState
    {
    public:
        PartialDismemberedLimbState(ComponentPtr _limb_component_ptr);

        void update(float dt) override;

        void on_enter() override;
        void on_exit() override;
    };
}