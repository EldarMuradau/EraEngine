#include "physics/physical_animation/limb_states/ragdoll_limb_state.h"
#include "physics/physical_animation/physical_animation_component.h"
#include "physics/physical_animation/physical_animation_utils.h"

namespace era_engine::physics
{
    RagdollLimbState::RagdollLimbState(ComponentPtr _limb_component_ptr)
        : BaseLimbState(_limb_component_ptr)
    {
    }

    void RagdollLimbState::on_enter()
    {
        ASSERT(!physical_animation_limb_component_ptr.is_empty());

        PhysicalAnimationLimbComponent* limb_component = static_cast<PhysicalAnimationLimbComponent*>(physical_animation_limb_component_ptr.get_for_write());

        PhysicalAnimationUtils::set_motor_drive_active(limb_component, false);

        BaseLimbState::on_enter();
    }
}