#include "physics/physical_animation/limb_states/simulation_limb_state.h"
#include "physics/physical_animation/physical_animation_component.h"
#include "physics/body_component.h"
#include "physics/joint.h"
#include "physics/shape_utils.h"
#include "physics/shape_component.h"
#include "physics/core/physics_utils.h"
#include "physics/physical_animation/physical_animation_utils.h"
#include "physics/physical_animation/ragdoll_profile.h"

#include <animation/animation.h>

#include <core/debug/debug_var.h>
#include <core/traits.h>

#include <ecs/base_components/transform_component.h>

namespace era_engine::physics
{
    SimulationLimbState::SimulationLimbState(ComponentPtr _limb_component_ptr)
        : BaseLimbState(_limb_component_ptr)
    {
    }

    void SimulationLimbState::update(float dt)
    {
        BaseLimbState::update(dt);

        PhysicalAnimationLimbComponent* limb_component = static_cast<PhysicalAnimationLimbComponent*>(physical_animation_limb_component_ptr.get_for_write());

        Entity limb = limb_component->get_entity();

        Entity ragdoll = limb_component->ragdoll_ptr.get();

        const PhysicalAnimationComponent* physical_animation_component = ragdoll.get_component<PhysicalAnimationComponent>();

        const ref<RagdollProfile> profile = physical_animation_component->get_ragdoll_profile();
        ASSERT(profile != nullptr);

        const PhysicalLimbDetails& limb_details = profile->get_limb_details_by_type(limb_component->type);

        // Velocity drag forces for stabilization (keyframe controller). Poses are in ragdoll space, velocities in world space.
        if (limb_details.drag_force.has_value())
        {
            const DragForceDetails& drag_details = limb_details.drag_force.value();
            DynamicBodyComponent* dynamic_body = limb.get_component<DynamicBodyComponent>();

            const trs& ragdoll_world_transform = ragdoll.get_component<TransformComponent>()->get_world_transform();
            const trs physics_world_pose = ragdoll_world_transform * limb_component->physics_pose;
            const trs target_world_pose = ragdoll_world_transform * limb_component->target_pose;

            const vec3 current_angular_velocity = dynamic_body->angular_velocity.get();

            const quat delta_rotation = normalize(target_world_pose.rotation * conjugate(physics_world_pose.rotation));
            vec3 desired_angular_velocity = get_shortest_angular_velocity(delta_rotation, dt);
            const float angular_velocity_magnitude = length(desired_angular_velocity);
            if (angular_velocity_magnitude > drag_details.partial_angular_drive_limit)
            {
                desired_angular_velocity *= (drag_details.partial_angular_drive_limit / angular_velocity_magnitude);
            }

            const vec3 drive_angular_velocity = lerp(current_angular_velocity, desired_angular_velocity, drag_details.partial_angular_drive);

            // Poses track the joint, physics velocities are at the centre of mass.
            const vec3 center_of_mass_offset = physics_world_pose.rotation * limb_component->mass_properties.center_of_mass;
            const vec3 current_origin_velocity = dynamic_body->linear_velocity.get() - cross(current_angular_velocity, center_of_mass_offset);

            vec3 desired_velocity = (target_world_pose.position - physics_world_pose.position) / dt;
            const float desired_velocity_magnitude = length(desired_velocity);
            if (desired_velocity_magnitude > drag_details.partial_velocity_drive_limit)
            {
                desired_velocity = desired_velocity * (drag_details.partial_velocity_drive_limit / desired_velocity_magnitude);
            }
            vec3 drive_origin_velocity = lerp(current_origin_velocity, desired_velocity, drag_details.partial_velocity_drive);

            vec3 root_velocity = physical_animation_component->velocity;
            const float root_velocity_magnitude = length(root_velocity);
            if (root_velocity_magnitude > drag_details.acceleration_limit)
            {
                root_velocity = root_velocity * (drag_details.acceleration_limit / root_velocity_magnitude);
            }
            drive_origin_velocity += root_velocity * drag_details.acceleration_gain;

            dynamic_body->angular_velocity = drive_angular_velocity;
            dynamic_body->linear_velocity = drive_origin_velocity + cross(drive_angular_velocity, center_of_mass_offset);
        }
    }

    PhysicalLimbStateType SimulationLimbState::try_switch_to(PhysicalLimbStateType desired_state) const
    {
        if (desired_state == PhysicalLimbStateType::KINEMATIC)
        {
            return PhysicalLimbStateType::BLEND_OUT;
        }
        return desired_state;
    }

    void SimulationLimbState::on_enter()
    {
        ASSERT(!physical_animation_limb_component_ptr.is_empty());

        PhysicalAnimationLimbComponent* limb_component = static_cast<PhysicalAnimationLimbComponent*>(physical_animation_limb_component_ptr.get_for_write());

        PhysicalAnimationUtils::reset_motor_drive(limb_component);
        PhysicalAnimationUtils::set_motor_drive_active(limb_component, true);

        BaseLimbState::on_enter();
    }
}