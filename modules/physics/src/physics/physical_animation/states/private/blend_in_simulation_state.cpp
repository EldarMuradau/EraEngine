#include "physics/physical_animation/states/blend_in_simulation_state.h"
#include "physics/physical_animation/physical_animation_component.h"
#include "physics/physical_animation/physical_animation_utils.h"
#include "physics/body_component.h"
#include "physics/joint.h"
#include "physics/shape_utils.h"
#include "physics/shape_component.h"

#include <ecs/base_components/transform_component.h>

#include <animation/animation.h>

namespace era_engine::physics
{
    BlendInSimulationState::BlendInSimulationState(ComponentPtr _physical_animation_component_ptr)
        : BaseSimulationState(_physical_animation_component_ptr)
    {
    }

    SimulationStateType BlendInSimulationState::try_switch_to(SimulationStateType desired_type) const
    {
        ASSERT(physical_animation_component_ptr.get() != nullptr);

        const PhysicalAnimationComponent* physical_animation_component = dynamic_cast<const PhysicalAnimationComponent*>(physical_animation_component_ptr.get());

        if (desired_type == SimulationStateType::DISABLED)
        {
            return SimulationStateType::BLEND_OUT;
        }
        else if (desired_type == SimulationStateType::RAGDOLL)
        {
            return SimulationStateType::RAGDOLL;
        }
        else if (desired_type == SimulationStateType::ENABLED &&
            blend_time >= physical_animation_component->blend_in_time)
        {
            return SimulationStateType::ENABLED;
        }
        return SimulationStateType::BLEND_IN;
    }

    void BlendInSimulationState::update(float dt)
    {
        blend_time += dt;
        blend_weight = map_value(blend_time, time_range.x, time_range.y, weight_range.x, weight_range.y);

        ASSERT(physical_animation_component_ptr.get() != nullptr);

        PhysicalAnimationComponent* physical_animation_component = dynamic_cast<PhysicalAnimationComponent*>(physical_animation_component_ptr.get_for_write());
        physical_animation_component->blend_weight = blend_weight;

        BaseSimulationState::update(dt);
    }

    void BlendInSimulationState::on_enter()
    {
        using namespace animation;

        ASSERT(physical_animation_component_ptr.get() != nullptr);

        PhysicalAnimationComponent* physical_animation_component = dynamic_cast<PhysicalAnimationComponent*>(physical_animation_component_ptr.get_for_write());

        blend_time = physical_animation_component->blend_weight * physical_animation_component->blend_in_time;
        time_range = vec2(0.0f, physical_animation_component->blend_in_time);

        PhysicalAnimationUtils::set_attachment_active(physical_animation_component, true);

		if (!physical_animation_component->simulated)
		{
			const trs& ragdoll_transform = physical_animation_component->get_entity().get_component<TransformComponent>()->get_world_transform();
			const SkeletonComponent* skeleton_component = physical_animation_component->get_entity().get_component<SkeletonComponent>();

            physical_animation_component->attachment_body.get().get_component<DynamicBodyComponent>()->simulated = true;

			auto process_limb = [&skeleton_component, &ragdoll_transform](PhysicalAnimationLimbComponent* limb_component)
				{
                    if (limb_component->get_current_state_type() == PhysicalLimbStateType::DISMEMBERED ||
                        limb_component->get_current_state_type() == PhysicalLimbStateType::PARTIAL_DISMEMBERED)
                    {
                        return;
                    }
					PhysicalAnimationUtils::set_simulation_for_limb(limb_component, true, true);
					PhysicalAnimationUtils::force_sync_limb_to_skeleton(limb_component, skeleton_component, ragdoll_transform);
				};

			traverse_simulation_graph(process_limb);

            physical_animation_component->simulated = true;

            AnimationComponent* animation_component = physical_animation_component->get_entity().get_component<AnimationComponent>();
            animation_component->update_skeleton = false;
        }

        BaseSimulationState::on_enter();
    }
}