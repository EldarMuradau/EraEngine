#include "physics/physical_animation/states/ragdoll_simulation_state.h"
#include "physics/physical_animation/physical_animation_component.h"
#include "physics/physical_animation/physical_animation_utils.h"
#include "physics/body_component.h"
#include "physics/joint.h"
#include "physics/core/physics_utils.h"
#include "physics/shape_utils.h"
#include "physics/shape_component.h"

#include <ecs/base_components/transform_component.h>

#include <animation/animation.h>

namespace era_engine::physics
{
    RagdollSimulationState::RagdollSimulationState(ComponentPtr _physical_animation_component_ptr)
        : BaseSimulationState(_physical_animation_component_ptr)
    {
    }

    SimulationStateType RagdollSimulationState::try_switch_to(SimulationStateType desired_type) const
    {
        return desired_type;
    }

    void RagdollSimulationState::update(float dt)
    {
        BaseSimulationState::update(dt);
    }

    void RagdollSimulationState::on_enter()
    {
        using namespace animation;

        ASSERT(physical_animation_component_ptr.get() != nullptr);

		PhysicalAnimationComponent* physical_animation_component = dynamic_cast<PhysicalAnimationComponent*>(physical_animation_component_ptr.get_for_write());
		Entity ragdoll = physical_animation_component->get_entity();

		const trs& ragdoll_transform = ragdoll.get_component<TransformComponent>()->get_world_transform();

		physical_animation_component->blend_weight = 1.0f;

		PhysicalAnimationUtils::set_attachment_active(physical_animation_component, false);
		const SkeletonComponent* skeleton_component = ragdoll.get_component<SkeletonComponent>();

		auto process_limb = [&skeleton_component, &ragdoll_transform](PhysicalAnimationLimbComponent* limb_component)
			{
				if (limb_component->get_current_state_type() == PhysicalLimbStateType::DISMEMBERED ||
					limb_component->get_current_state_type() == PhysicalLimbStateType::PARTIAL_DISMEMBERED)
				{
					return;
				}
				PhysicalAnimationUtils::force_sync_limb_to_skeleton(limb_component, skeleton_component, ragdoll_transform);
				PhysicalAnimationUtils::set_simulation_for_limb(limb_component, true, true);
			};

		traverse_simulation_graph(process_limb);

		AnimationComponent* animation_component = physical_animation_component->get_entity().get_component<AnimationComponent>();
		animation_component->update_skeleton = false;
		animation_component->play = false;

		BaseSimulationState::on_enter();
    }

    void RagdollSimulationState::on_exit()
    {
		using namespace animation;

		ASSERT(physical_animation_component_ptr.get() != nullptr);
		PhysicalAnimationComponent* physical_animation_component = dynamic_cast<PhysicalAnimationComponent*>(physical_animation_component_ptr.get_for_write());

		Entity ragdoll = physical_animation_component->get_entity();

		const trs& ragdoll_transform = ragdoll.get_component<TransformComponent>()->get_world_transform();

		PhysicalAnimationUtils::set_attachment_active(physical_animation_component, true);

		AnimationComponent* animation_component = ragdoll.get_component<AnimationComponent>();
		animation_component->play = true;

		const SkeletonComponent* skeleton_component = ragdoll.get_component<SkeletonComponent>();

		trs pelvis_local_transform = SkeletonUtils::get_object_space_joint_transform(animation_component->current_animation_pose, skeleton_component->skeleton.get(), physical_animation_component->root_joint_id, 0);
		pelvis_local_transform.scale = vec3(1.0f);

		const trs pelvis_world_transform = ragdoll_transform * pelvis_local_transform;
		PhysicsUtils::manual_set_physics_transform_locked(physical_animation_component->attachment_body.get(), pelvis_world_transform, true);

		for (const EntityPtr& limb_ptr : physical_animation_component->limbs)
		{
			Entity limb = limb_ptr.get();
			PhysicalAnimationLimbComponent* limb_data_component = limb.get_component<PhysicalAnimationLimbComponent>();

			if (limb_data_component->get_current_state_type() == PhysicalLimbStateType::DISMEMBERED ||
				limb_data_component->get_current_state_type() == PhysicalLimbStateType::PARTIAL_DISMEMBERED)
			{
				return;
			}

			ASSERT(limb_data_component != nullptr);

			if (limb_data_component->drive_joint_component.get() != nullptr)
			{
				D6JointComponent* drive_joint = dynamic_cast<D6JointComponent*>(limb_data_component->drive_joint_component.get_for_write());
				PhysicsUtils::manual_set_physics_transform_locked(drive_joint->get_first_entity_ptr().get(), ragdoll_transform * limb_data_component->target_pose, true);
			}
		}

		BaseSimulationState::on_exit();
	}
}