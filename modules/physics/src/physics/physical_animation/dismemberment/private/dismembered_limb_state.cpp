#include "physics/physical_animation/dismemberment/dismembered_limb_state.h"
#include "physics/physical_animation/physical_animation_component.h"
#include "physics/physical_animation/physical_animation_utils.h"
#include "physics/body_component.h"
#include "physics/joint.h"
#include "physics/shape_utils.h"
#include "physics/shape_component.h"
#include "physics/core/physics_utils.h"

#include <animation/animation.h>

#include <ecs/base_components/transform_component.h>

namespace era_engine::physics
{
    DismemberedLimbState::DismemberedLimbState(ComponentPtr _limb_component_ptr)
        : BaseLimbState(_limb_component_ptr)
    {
    }

    void DismemberedLimbState::update(float dt)
    {
        BaseLimbState::update(dt);
    }

    void DismemberedLimbState::on_enter()
    {
        ASSERT(!physical_animation_limb_component_ptr.is_empty());

        PhysicalAnimationLimbComponent* limb_component = static_cast<PhysicalAnimationLimbComponent*>(physical_animation_limb_component_ptr.get_for_write());

        PhysicalAnimationUtils::reset_motor_drive(limb_component);
        PhysicalAnimationUtils::set_limb_kinematic(limb_component, false);
        PhysicalAnimationUtils::set_simulation_for_limb(limb_component, true, true);
        PhysicalAnimationUtils::set_full_constraint_active(limb_component, false);

        //limb_component->apply_impulse(limb_component->collision.impulse);

        BaseLimbState::on_enter();
    }

    void DismemberedLimbState::on_exit()
    {
        BaseLimbState::on_exit();
    }

    PartialDismemberedLimbState::PartialDismemberedLimbState(ComponentPtr _limb_component_ptr)
        : BaseLimbState(_limb_component_ptr)
    {
    }

    void PartialDismemberedLimbState::update(float dt)
    {
        BaseLimbState::update(dt);
    }

    void PartialDismemberedLimbState::on_enter()
    {
        ASSERT(!physical_animation_limb_component_ptr.is_empty());

        PhysicalAnimationLimbComponent* limb_component = static_cast<PhysicalAnimationLimbComponent*>(physical_animation_limb_component_ptr.get_for_write());

        PhysicalAnimationUtils::reset_motor_drive(limb_component);
        PhysicalAnimationUtils::set_limb_kinematic(limb_component, false);
        PhysicalAnimationUtils::set_motor_drive_active(limb_component, false);
        PhysicalAnimationUtils::set_simulation_for_limb(limb_component, true, true);

        //limb_component->apply_impulse(limb_component->collision.impulse);

        BaseLimbState::on_enter();
    }

    void PartialDismemberedLimbState::on_exit()
    {
        BaseLimbState::on_exit();
    }
}