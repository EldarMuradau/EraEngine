#include "physics/physical_animation/limb_states/kinematic_limb_state.h"
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
    KinematicLimbState::KinematicLimbState(ComponentPtr _limb_component_ptr)
        : BaseLimbState(_limb_component_ptr)
    {
    }

    void KinematicLimbState::update(float dt)
    {
        BaseLimbState::update(dt);

        PhysicalAnimationLimbComponent* limb_component = static_cast<PhysicalAnimationLimbComponent*>(physical_animation_limb_component_ptr.get_for_write());

        Entity limb = limb_component->get_entity();

        Entity ragdoll = limb_component->ragdoll_ptr.get();
        const trs& ragdoll_world_transform = ragdoll.get_component<TransformComponent>()->get_world_transform();

        const trs desired_pose = ragdoll_world_transform * limb_component->target_pose;

        PhysicsUtils::manual_set_physics_transform_locked(limb, desired_pose, true);
    }

    void KinematicLimbState::on_enter()
    {
        ASSERT(!physical_animation_limb_component_ptr.is_empty());

        PhysicalAnimationLimbComponent* limb_component = static_cast<PhysicalAnimationLimbComponent*>(physical_animation_limb_component_ptr.get_for_write());

        PhysicalAnimationUtils::reset_motor_drive(limb_component);
        PhysicalAnimationUtils::set_motor_drive_active(limb_component, false);
        PhysicalAnimationUtils::set_limb_kinematic(limb_component, true);

        BaseLimbState::on_enter();
    }

    void KinematicLimbState::on_exit()
    {
        ASSERT(!physical_animation_limb_component_ptr.is_empty());

        PhysicalAnimationLimbComponent* limb_component = static_cast<PhysicalAnimationLimbComponent*>(physical_animation_limb_component_ptr.get_for_write());

        PhysicalAnimationUtils::set_limb_kinematic(limb_component, false);

        BaseLimbState::on_enter();
    }
}