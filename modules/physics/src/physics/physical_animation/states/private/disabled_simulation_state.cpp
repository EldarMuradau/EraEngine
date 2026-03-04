#include "physics/physical_animation/states/disabled_simulation_state.h"
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
    DisabledSimulationState::DisabledSimulationState(ComponentPtr _physical_animation_component_ptr)
        : BaseSimulationState(_physical_animation_component_ptr)
    {
    }

    SimulationStateType DisabledSimulationState::try_switch_to(SimulationStateType desired_type) const
    {
        if (desired_type == SimulationStateType::ENABLED)
        {
            return SimulationStateType::BLEND_IN;
        }
        else if (desired_type == SimulationStateType::RAGDOLL)
        {
            return SimulationStateType::RAGDOLL;
        }
        return SimulationStateType::DISABLED;
    }

    void DisabledSimulationState::on_enter()
    {
        using namespace animation;

        ASSERT(physical_animation_component_ptr.get() != nullptr);

        PhysicalAnimationComponent* physical_animation_component = dynamic_cast<PhysicalAnimationComponent*>(physical_animation_component_ptr.get_for_write());
        physical_animation_component->reset_blends();

        if (physical_animation_component->simulated)
        {
            auto process_limb = [](PhysicalAnimationLimbComponent* limb_component)
                {
                    limb_component->reset_collision_data();
                    limb_component->force_switch_state(PhysicalLimbStateType::KINEMATIC);

                    PhysicalAnimationUtils::reset_motor_drive(limb_component);
                    PhysicalAnimationUtils::set_simulation_for_limb(limb_component, false, false);
                };

            traverse_simulation_graph(process_limb);

            physical_animation_component->simulated = false;

            physical_animation_component->attachment_body.get().get_component<DynamicBodyComponent>()->simulated = false;

            AnimationComponent* animation_component = physical_animation_component->get_entity().get_component<AnimationComponent>();
            animation_component->update_skeleton = true;
        }

        BaseSimulationState::on_enter();
    }

    void DisabledSimulationState::update(float dt)
    {
        using namespace animation;

        Entity ragdoll = physical_animation_component_ptr.get_entity();
        const AnimationComponent* animation_component = ragdoll.get_component<AnimationComponent>();

        if (animation_component->current_animation == nullptr)
        {
            return;
        }

        const SkeletonComponent* skeleton_component = ragdoll.get_component<SkeletonComponent>();
        const trs& ragdoll_transform = ragdoll.get_component<TransformComponent>()->get_world_transform();

        auto process_limb = [&skeleton_component, &ragdoll_transform](PhysicalAnimationLimbComponent* limb_component)
            {
                PhysicalAnimationUtils::force_sync_limb_to_skeleton(limb_component, skeleton_component, ragdoll_transform);
            };

        traverse_simulation_graph(process_limb);
    }
}