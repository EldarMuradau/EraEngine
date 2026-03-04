#include "physics/physical_animation/physical_animation_system.h"
#include "physics/core/physics.h"
#include "physics/core/physics_utils.h"
#include "physics/body_component.h"
#include "physics/joint.h"
#include "physics/core/physics_utils.h"
#include "physics/shape_utils.h"
#include "physics/collisions_holder_root_component.h"
#include "physics/physical_animation/physical_animation_builder.h"
#include "physics/scene_queries.h"
#include "physics/shape_component.h"
#include "physics/physical_animation/ragdoll_profile.h"
#include "physics/physical_animation/physical_animation_utils.h"

#include <core/cpu_profiling.h>
#include <core/debug/debug_var.h>
#include <core/log.h>
#include <core/traits.h>

#include <rendering/debug_visualization.h>
#include <rendering/ecs/renderer_holder_root_component.h>

#include <ecs/update_groups.h>

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<PhysicalAnimationSystem>("PhysicalAnimationSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("physics")))

			.method("update_before_physics", &PhysicalAnimationSystem::update_before_physics)
			(metadata("update_group", update_types::PHYSICS),
			 metadata("Before", std::vector<std::string>{"PhysicsSystem::update"}))

			.method("update_after_physics", &PhysicalAnimationSystem::update_after_physics)
			(metadata("update_group", update_types::PHYSICS),
				metadata("After", std::vector<std::string>{"PhysicsSystem::update"}))

			.method("update_normal", &PhysicalAnimationSystem::update_normal)
			(metadata("update_group", update_types::GAMEPLAY_NORMAL),
			metadata("After", std::vector<std::string>{"AnimationSystem::update"}));
	}

	static DebugVar<bool> enable_always = DebugVar<bool>("physics.physical_animation.enable_always", false);
	static DebugVar<bool> force_ragdoll = DebugVar<bool>("physics.physical_animation.force_ragdoll", false);

	static DebugVar<float> running_speed = DebugVar<float>("physics.physical_animation.debug_demo.running_speed", 1.0f);
	static DebugVar<float> sprint_speed = DebugVar<float>("physics.physical_animation.debug_demo.sprint_speed", 5.0f);

	PhysicalAnimationSystem::PhysicalAnimationSystem(World* _world)
		: System(_world)
	{
	}

	void PhysicalAnimationSystem::init()
	{
		using namespace animation;
		entt::registry& registry = world->get_registry();
		registry.on_construct<PhysicalAnimationComponent>().connect<&PhysicalAnimationSystem::on_pac_created>(this);

		ragdolls_group = world->group(components_group<TransformComponent, PhysicalAnimationComponent, AnimationComponent, SkeletonComponent>);

		collisions_holder_rc = world->add_root_component<CollisionsHolderRootComponent>();
		renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();

		{
			idle_profile = make_ref<RagdollProfile>();

			idle_profile->type = RagdollProfileType::IDLE;
			idle_profile->head_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			idle_profile->head_limb_details.motor_drive = MotorDriveDetails();
			idle_profile->head_limb_details.motor_drive->angular_drive_stiffness = 300.0f;
			idle_profile->head_limb_details.motor_drive->linear_drive_stiffness = 300.0f;

			idle_profile->neck_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			idle_profile->neck_limb_details.motor_drive = MotorDriveDetails();
			idle_profile->neck_limb_details.motor_drive->angular_drive_stiffness = 300.0f;
			idle_profile->neck_limb_details.motor_drive->linear_drive_stiffness = 300.0f;

			idle_profile->body_upper_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			idle_profile->body_upper_limb_details.motor_drive = MotorDriveDetails();
			idle_profile->body_upper_limb_details.motor_drive->angular_drive_stiffness = 500.0f;
			idle_profile->body_upper_limb_details.motor_drive->linear_drive_stiffness = 400.0f;

			idle_profile->body_middle_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			idle_profile->body_middle_limb_details.motor_drive = MotorDriveDetails();
			idle_profile->body_middle_limb_details.motor_drive->angular_drive_stiffness = 500.0f;
			idle_profile->body_middle_limb_details.motor_drive->linear_drive_stiffness = 400.0f;

			idle_profile->body_lower_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;

			idle_profile->clavicle_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			idle_profile->clavicle_limb_details.motor_drive = MotorDriveDetails();
			idle_profile->clavicle_limb_details.motor_drive->angular_drive_stiffness = 200.0f;
			idle_profile->clavicle_limb_details.motor_drive->linear_drive_stiffness = 400.0f;

			idle_profile->arm_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			idle_profile->arm_limb_details.motor_drive = MotorDriveDetails();
			idle_profile->arm_limb_details.motor_drive->angular_drive_stiffness = 200.0f;
			idle_profile->arm_limb_details.motor_drive->linear_drive_stiffness = 400.0f;

			idle_profile->forearm_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			idle_profile->forearm_limb_details.motor_drive = MotorDriveDetails();
			idle_profile->forearm_limb_details.motor_drive->angular_drive_stiffness = 200.0f;
			idle_profile->forearm_limb_details.motor_drive->linear_drive_stiffness = 400.0f;

			idle_profile->hand_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			idle_profile->hand_limb_details.motor_drive = MotorDriveDetails();
			idle_profile->hand_limb_details.motor_drive->angular_drive_stiffness = 200.0f;
			idle_profile->hand_limb_details.motor_drive->linear_drive_stiffness = 400.0f;

			idle_profile->leg_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			idle_profile->leg_limb_details.motor_drive = MotorDriveDetails();
			idle_profile->leg_limb_details.motor_drive->angular_drive_stiffness = 400.0f;
			idle_profile->leg_limb_details.motor_drive->linear_drive_stiffness = 300.0f;

			idle_profile->calf_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			idle_profile->calf_limb_details.motor_drive = MotorDriveDetails();
			idle_profile->calf_limb_details.motor_drive->angular_drive_stiffness = 400.0f;
			idle_profile->calf_limb_details.motor_drive->linear_drive_stiffness = 300.0f;

			idle_profile->foot_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			idle_profile->foot_limb_details.motor_drive = MotorDriveDetails();
			idle_profile->foot_limb_details.motor_drive->angular_drive_stiffness = 400.0f;
			idle_profile->foot_limb_details.motor_drive->linear_drive_stiffness = 300.0f;
		}

		{
			running_profile = make_ref<RagdollProfile>();

			running_profile->type = RagdollProfileType::RUNNING;
			running_profile->head_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			running_profile->head_limb_details.motor_drive = MotorDriveDetails();
			running_profile->head_limb_details.motor_drive->angular_drive_stiffness = 300.0f;
			running_profile->head_limb_details.motor_drive->linear_drive_stiffness = 300.0f;

			running_profile->neck_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			running_profile->neck_limb_details.motor_drive = MotorDriveDetails();
			running_profile->neck_limb_details.motor_drive->angular_drive_stiffness = 300.0f;
			running_profile->neck_limb_details.motor_drive->linear_drive_stiffness = 300.0f;

			running_profile->body_upper_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			running_profile->body_upper_limb_details.motor_drive = MotorDriveDetails();
			running_profile->body_upper_limb_details.motor_drive->angular_drive_stiffness = 500.0f;
			running_profile->body_upper_limb_details.motor_drive->linear_drive_stiffness = 400.0f;

			running_profile->body_middle_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			running_profile->body_middle_limb_details.motor_drive = MotorDriveDetails();
			running_profile->body_middle_limb_details.motor_drive->angular_drive_stiffness = 500.0f;
			running_profile->body_middle_limb_details.motor_drive->linear_drive_stiffness = 400.0f;

			running_profile->body_lower_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;

			running_profile->clavicle_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			running_profile->clavicle_limb_details.motor_drive = MotorDriveDetails();
			running_profile->clavicle_limb_details.motor_drive->angular_drive_stiffness = 200.0f;
			running_profile->clavicle_limb_details.motor_drive->linear_drive_stiffness = 400.0f;

			running_profile->arm_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			running_profile->arm_limb_details.motor_drive = MotorDriveDetails();
			running_profile->arm_limb_details.motor_drive->angular_drive_stiffness = 200.0f;
			running_profile->arm_limb_details.motor_drive->linear_drive_stiffness = 400.0f;

			running_profile->forearm_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			running_profile->forearm_limb_details.motor_drive = MotorDriveDetails();
			running_profile->forearm_limb_details.motor_drive->angular_drive_stiffness = 200.0f;
			running_profile->forearm_limb_details.motor_drive->linear_drive_stiffness = 400.0f;

			running_profile->hand_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			running_profile->hand_limb_details.motor_drive = MotorDriveDetails();
			running_profile->hand_limb_details.motor_drive->angular_drive_stiffness = 200.0f;
			running_profile->hand_limb_details.motor_drive->linear_drive_stiffness = 400.0f;

			running_profile->leg_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			running_profile->leg_limb_details.motor_drive = MotorDriveDetails();
			running_profile->leg_limb_details.motor_drive->angular_drive_stiffness = 400.0f;
			running_profile->leg_limb_details.motor_drive->linear_drive_stiffness = 300.0f;

			running_profile->calf_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			running_profile->calf_limb_details.motor_drive = MotorDriveDetails();
			running_profile->calf_limb_details.motor_drive->angular_drive_stiffness = 400.0f;
			running_profile->calf_limb_details.motor_drive->linear_drive_stiffness = 300.0f;

			running_profile->foot_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
			running_profile->foot_limb_details.motor_drive = MotorDriveDetails();
			running_profile->foot_limb_details.motor_drive->angular_drive_stiffness = 400.0f;
			running_profile->foot_limb_details.motor_drive->linear_drive_stiffness = 300.0f;
		}

		{
			sprint_profile = make_ref<RagdollProfile>();

			sprint_profile->type = RagdollProfileType::SPRINT;
			sprint_profile->head_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_ANIMATION_POSE;
			sprint_profile->arm_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_ANIMATION_POSE;
			sprint_profile->body_upper_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_ANIMATION_POSE;
			sprint_profile->body_middle_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_ANIMATION_POSE;
			sprint_profile->forearm_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_ANIMATION_POSE;
			sprint_profile->hand_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_ANIMATION_POSE;
			sprint_profile->leg_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_ANIMATION_POSE;
			sprint_profile->calf_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_ANIMATION_POSE;
			sprint_profile->foot_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_ANIMATION_POSE;
		}
		
		{
			ragdoll_profile = make_ref<RagdollProfile>();

			ragdoll_profile->type = RagdollProfileType::RAGDOLL;
			ragdoll_profile->head_limb_details.blend_type = PhysicalLimbBlendType::PURE_PHYSICS;
			ragdoll_profile->arm_limb_details.blend_type = PhysicalLimbBlendType::PURE_PHYSICS;
			ragdoll_profile->body_upper_limb_details.blend_type = PhysicalLimbBlendType::PURE_PHYSICS;
			ragdoll_profile->body_middle_limb_details.blend_type = PhysicalLimbBlendType::PURE_PHYSICS;
			ragdoll_profile->forearm_limb_details.blend_type = PhysicalLimbBlendType::PURE_PHYSICS;
			ragdoll_profile->hand_limb_details.blend_type = PhysicalLimbBlendType::PURE_PHYSICS;
			ragdoll_profile->leg_limb_details.blend_type = PhysicalLimbBlendType::PURE_PHYSICS;
			ragdoll_profile->calf_limb_details.blend_type = PhysicalLimbBlendType::PURE_PHYSICS;
			ragdoll_profile->foot_limb_details.blend_type = PhysicalLimbBlendType::PURE_PHYSICS;
		}

		pose_sampler = std::make_unique<DrivePoseSampler>();
	}

	void PhysicalAnimationSystem::update_before_physics(float dt)
	{
		ZoneScopedN("PhysicalAnimationSystem::update_before_physics");

		process_added_pacs();

		for (auto&& [entity_handle, changed_flag, physical_animation_component] : world->group(components_group<TransformComponent, PhysicalAnimationComponent>).each())
		{
			if (physical_animation_component.simulated.is_changed())
			{
				for (const EntityPtr& limb_ptr : physical_animation_component.limbs)
				{
					Entity limb = limb_ptr.get();
					limb.get_component<PhysicalAnimationLimbComponent>()->simulated = physical_animation_component.simulated;
				}
				physical_animation_component.simulated.sync_changes();
			}
		}

		for (auto&& [entity_handle, changed_flag, physical_animation_limb_component] : world->group(components_group<TransformComponent, PhysicalAnimationLimbComponent>).each())
		{
			if (physical_animation_limb_component.simulated.is_changed())
			{
				Entity limb = world->get_entity(entity_handle);
				limb.get_component<DynamicBodyComponent>()->simulated = physical_animation_limb_component.simulated;
				physical_animation_limb_component.simulated.sync_changes();
			}
		}

		for (auto&& [entity_handle, transform_component, physical_animation_component, animm, skeleton] : ragdolls_group.each())
		{
			if (physical_animation_component.loaded)
			{
				continue;
			}
			bool is_ready_for_simulation = true;
			for (const EntityPtr& limb_ptr : physical_animation_component.limbs)
			{
				Entity limb = limb_ptr.get();

				ShapeComponent* shape_component = ShapeUtils::get_shape_component(limb);
				DynamicBodyComponent* body_component = limb.get_component<DynamicBodyComponent>();

				if (body_component != nullptr && shape_component != nullptr)
				{
					is_ready_for_simulation &= body_component->get_rigid_actor() != nullptr;
					is_ready_for_simulation &= shape_component->get_shape() != nullptr;
					if (body_component->get_rigid_actor() != nullptr)
					{
						is_ready_for_simulation &= body_component->get_rigid_actor()->userData != nullptr;
					}
					if (shape_component->get_shape() != nullptr)
					{
						is_ready_for_simulation &= shape_component->get_shape()->userData != nullptr;
					}
				}
				else
				{
					is_ready_for_simulation = false;
					break;
				}

				if (!is_ready_for_simulation)
				{
					break;
				}
			}

			physical_animation_component.loaded = is_ready_for_simulation;
		}

		update_ragdolls(dt);
	}

	void PhysicalAnimationSystem::update_after_physics(float dt)
	{
		ZoneScopedN("PhysicalAnimationSystem::update_after_physics");

		if (active_ragdolls.empty())
		{
			return;
		}

		for (PhysicalAnimationComponent* physical_animation_component : active_ragdolls)
		{
			for (const EntityPtr& limb_ptr : physical_animation_component->limbs)
			{
				Entity limb = limb_ptr.get();
				PhysicalAnimationLimbComponent* limb_data_component = limb.get_component<PhysicalAnimationLimbComponent>();

				ASSERT(limb_data_component != nullptr);

				limb_data_component->prev_physics_pose = limb_data_component->physics_pose;
				limb_data_component->physics_pose = limb.get_component<TransformComponent>()->get_local_transform();
			}

			physical_animation_component->reached_physics_pose = false;
			physical_animation_component->elapsed_blend_time = 0.0f;
		}

		filter_states_by_collisions(dt);

		for (PhysicalAnimationComponent* physical_animation_component : active_ragdolls)
		{
			update_chains_states(physical_animation_component, dt);
		}
	}

	void PhysicalAnimationSystem::update_normal(float dt)
	{
		using namespace animation;

		ZoneScopedN("PhysicalAnimationSystem::update_normal");

		for (auto&& [entity_handle,
			transform_component,
			physical_animation_component,
			animation_component,
			skeleton_component] : ragdolls_group.each())
		{
			if (!physical_animation_component.loaded)
			{
				continue;
			}

			const trs& world_transform = transform_component.get_world_transform();

			const vec3 root_delta_motion = world_transform.position - physical_animation_component.prev_world_transform.position;
			physical_animation_component.velocity = root_delta_motion / dt;

			physical_animation_component.prev_world_transform = world_transform;

			const bool is_simulated = physical_animation_component.get_current_state_type() != SimulationStateType::DISABLED;

			if (is_simulated)
			{
				ref<Skeleton> skeleton = skeleton_component.skeleton;
				if (skeleton == nullptr)
				{
					continue;
				}

				const SkeletonPose& current_animation_pose = animation_component.current_animation_pose;

				// Sample only if simulation enabled.
				pose_sampler->sample_pose(&physical_animation_component, &skeleton_component, current_animation_pose, dt);
			}
		}
	}

	void PhysicalAnimationSystem::update_ragdolls(float dt)
	{
		using namespace animation;

		ZoneScopedN("PhysicalAnimationSystem::update_ragdolls");

		active_ragdolls.clear();

		for (auto&& [entity_handle,
			transform_component,
			physical_animation_component,
			animation_component,
			skeleton_component] : ragdolls_group.each())
		{
			if (!physical_animation_component.loaded)
			{
				continue;
			}

			const ref<Skeleton> skeleton = skeleton_component.skeleton;
			if (skeleton == nullptr)
			{
				ASSERT(false);
				continue;
			}

			if (animation_component.current_animation == nullptr)
			{
				continue;
			}

			Entity simulated_body = world->get_entity(entity_handle);

			const trs& world_transform = transform_component.get_world_transform();

			const bool is_should_be_simulated = should_be_simulated(simulated_body);

			if (is_should_be_simulated) // Update profiles only if simulated.
			{
				update_ragdoll_profiles(&physical_animation_component, physical_animation_component.velocity, dt, false);
			}

			// Update states.
			{
				if (force_ragdoll)
				{
					physical_animation_component.force_set_ragdoll(true);
				}
				else
				{
					physical_animation_component.force_set_ragdoll(false);
				}

				SimulationStateType desired_state_type = SimulationStateType::DISABLED;
				if (physical_animation_component.is_in_ragdoll())
				{
					desired_state_type = SimulationStateType::RAGDOLL;
				}
				else if (is_should_be_simulated)
				{
					desired_state_type = SimulationStateType::ENABLED;
				}

				physical_animation_component.update_states(dt, desired_state_type);
			}

			const SimulationStateType current_state_type = physical_animation_component.get_current_state_type();
			const bool is_disabled = current_state_type == SimulationStateType::DISABLED;

			if (is_disabled)
			{
				continue;
			}

			active_ragdolls.emplace_back(&physical_animation_component);

			const SkeletonPose& current_animation_pose = animation_component.current_animation_pose;
			update_target_pose(&physical_animation_component, current_animation_pose, skeleton.get(), world_transform);

			for (const EntityPtr& limb_ptr : physical_animation_component.limbs)
			{
				Entity limb = limb_ptr.get();
				PhysicalAnimationLimbComponent* limb_data_component = limb.get_component<PhysicalAnimationLimbComponent>();

				ASSERT(limb_data_component != nullptr);

				limb_data_component->is_colliding = false;
			}
		}
	}

	void PhysicalAnimationSystem::filter_states_by_collisions(float dt) const
	{
		ref<SimulationEventCallback> event_callback = PhysicsHolder::physics_ref->simulation_event_callback;
		for (const Collision& collision : event_callback->new_collisions)
		{
			Entity first_entity = collision.this_actor->get_entity();
			Entity second_entity = collision.other_actor->get_entity();

			if (first_entity == second_entity)
			{
				continue;
			}

			if (first_entity.get_parent_handle() == second_entity.get_parent_handle())
			{
				continue;
			}

			PhysicalAnimationLimbComponent* first_limb_data_component = first_entity.get_component_if_exists<PhysicalAnimationLimbComponent>();
			if (first_limb_data_component != nullptr)
			{
				Entity first_ragdoll = first_limb_data_component->ragdoll_ptr.get();
				const PhysicalAnimationComponent* first_simulation_component = first_ragdoll.get_component<PhysicalAnimationComponent>();
				ASSERT(first_simulation_component != nullptr);

				const SimulationStateType state = first_simulation_component->get_current_state_type();
				if((state == SimulationStateType::ENABLED || 
					state == SimulationStateType::RAGDOLL) &&
					first_limb_data_component->type != RagdollLimbType::FOOT)
				{
					first_limb_data_component->is_colliding = true;
				}
			}

			PhysicalAnimationLimbComponent* second_limb_data_component = second_entity.get_component_if_exists<PhysicalAnimationLimbComponent>();
			if (second_limb_data_component != nullptr)
			{
				Entity second_ragdoll = second_limb_data_component->ragdoll_ptr.get();
				const PhysicalAnimationComponent* second_simulation_component = second_ragdoll.get_component<PhysicalAnimationComponent>();
				ASSERT(second_simulation_component != nullptr);

				const SimulationStateType state = second_simulation_component->get_current_state_type();
				if ((state == SimulationStateType::ENABLED ||
					state == SimulationStateType::RAGDOLL) &&
					second_limb_data_component->type != RagdollLimbType::FOOT)
				{
					second_limb_data_component->is_colliding = true;
				}
			}
		}
	}

	void PhysicalAnimationSystem::update_chains_states(const PhysicalAnimationComponent* physical_animation_component, float dt) const
	{
		const bool is_in_ragdoll = physical_animation_component->is_in_ragdoll();

		update_chain(physical_animation_component->left_leg_chain, dt, is_in_ragdoll);
		update_chain(physical_animation_component->right_leg_chain, dt, is_in_ragdoll);
		update_chain(physical_animation_component->body_chain, dt, is_in_ragdoll);

		update_chain(physical_animation_component->neck_chain, dt, is_in_ragdoll);
		update_chain(physical_animation_component->left_arm_chain, dt, is_in_ragdoll);
		update_chain(physical_animation_component->right_arm_chain, dt, is_in_ragdoll);
	}

	void PhysicalAnimationSystem::update_chain(const ref<PhysicsLimbChain>& chain, float dt, bool is_in_ragdoll) const
	{
		for (EntityPtr limb_ptr : chain->connected_limbs)
		{
			Entity limb = limb_ptr.get();

			PhysicalAnimationLimbComponent* limb_data_component = limb.get_component<PhysicalAnimationLimbComponent>();
			ASSERT(limb_data_component != nullptr);

			PhysicalLimbStateType desired_chain_state = PhysicalLimbStateType::SIMULATION;
			if (is_in_ragdoll)
			{
				desired_chain_state = PhysicalLimbStateType::RAGDOLL;
			}

			limb_data_component->update_states(dt, desired_chain_state);

			if (limb_data_component->was_in_collision && limb_data_component->is_colliding)
			{
				const float used_collision_time = PhysicalAnimationUtils::is_arm_limb(limb_data_component->type) ?
					PhysicalAnimationLimbComponent::MAX_FREQUENT_COLLISION_TIME :
					PhysicalAnimationLimbComponent::MAX_COLLISION_TIME;
				limb_data_component->collision_time = clamp(limb_data_component->collision_time + dt, 0.0f, used_collision_time);
			}
			else
			{
				limb_data_component->collision_time = max(limb_data_component->collision_time - dt, 0.0f);
			}
			limb_data_component->was_in_collision = limb_data_component->is_colliding;
		}
	}

	void PhysicalAnimationSystem::update_target_pose(PhysicalAnimationComponent* physical_animation_component, 
		const animation::SkeletonPose& current_animation_pose,
		const animation::Skeleton* skeleton,
		const trs& ragdoll_world_space_pose)
	{
		auto simulated_joints_end = physical_animation_component->simulated_joints.end();

		for (const uint32 simulation_joint : physical_animation_component->simulated_joints_set)
		{
			if (simulation_joint == 0)
			{
				continue;
			}

			const uint32 parent_id = skeleton->joints.at(simulation_joint).parent_id;
			const trs& parent_local = physical_animation_component->local_joint_poses_for_target_calculation[parent_id];

			trs limb_animation_pose = current_animation_pose.get_joint_transform(simulation_joint).get_transform();
			limb_animation_pose.scale = vec3(1.0f);

			// Update targets from current animation.
			const trs calculated_transform = parent_local * limb_animation_pose;
			physical_animation_component->local_joint_poses_for_target_calculation[simulation_joint] = calculated_transform;

			auto limb_iter = physical_animation_component->simulated_joints.find(simulation_joint);
			if (limb_iter != simulated_joints_end)
			{
				Entity limb = limb_iter->second.get();

				PhysicalAnimationLimbComponent* limb_component = limb.get_component<PhysicalAnimationLimbComponent>();
				limb_component->target_pose = calculated_transform;

				if (limb_component->drive_joint_component.get() != nullptr)
				{
					D6JointComponent* drive_joint = dynamic_cast<D6JointComponent*>(limb_component->drive_joint_component.get_for_write());
					PhysicsUtils::manual_set_physics_transform(drive_joint->get_first_entity_ptr().get(), ragdoll_world_space_pose * limb_component->target_pose, true);
				}
			}
		}

		TransformComponent* root_attachment_transform_component = physical_animation_component->attachment_body.get().get_component<TransformComponent>();
		PhysicsUtils::manual_set_physics_transform(physical_animation_component->attachment_body.get(),
			ragdoll_world_space_pose * physical_animation_component->local_joint_poses_for_target_calculation[physical_animation_component->root_joint_id],
			true);
	}

	void PhysicalAnimationSystem::update_ragdoll_profiles(PhysicalAnimationComponent* physical_animation_component,
		const vec3& velocity,
		float dt,
		bool force_reload) const
	{
		ref<RagdollProfile> desired_profile;

		const float square_velocity_magnitude = squared_length(velocity);
		if (square_velocity_magnitude < running_speed * running_speed)
		{
			desired_profile = idle_profile;
		}
		else if (square_velocity_magnitude < sprint_speed * sprint_speed)
		{
			desired_profile = running_profile;
		}
		else
		{
			desired_profile = sprint_profile;
		}

		if (desired_profile != nullptr)
		{
			physical_animation_component->try_to_apply_ragdoll_profile(desired_profile, RagdollProfileStrengthType::DEFAULT, force_reload);
		}
		else
		{
			ASSERT(false);
		}
	}

	bool PhysicalAnimationSystem::should_be_simulated(Entity ragdoll) const
	{
		return enable_always;
	}

	void PhysicalAnimationSystem::process_added_pacs()
	{
		using namespace physx;
		using namespace animation;

		ScopedSpinLock _lock{ sync };

		for (Entity::Handle entity_handle : std::exchange(pacs_to_init, {}))
		{
			Entity entity = world->get_entity(entity_handle);

			const SkeletonComponent* skeleton_component = entity.get_component<SkeletonComponent>();

			const ref<Skeleton> skeleton = skeleton_component->skeleton;
			if (skeleton == nullptr)
			{
				continue;
			}

			PhysicalAnimationComponent* physical_animation_component = entity.get_component<PhysicalAnimationComponent>();
			physical_animation_component->current_profile = idle_profile;

			PhysicalAnimationBuilderUtils::build_simulated_body(entity);

			for (const EntityPtr& limb_ptr : physical_animation_component->limbs)
			{
				const PhysicalAnimationLimbComponent* limb_component = limb_ptr.get().get_component<PhysicalAnimationLimbComponent>();
				physical_animation_component->simulated_joints.emplace(limb_component->joint_id, limb_ptr);
			}

			physical_animation_component->root_joint_id = physical_animation_component->joint_init_ids.attachment_idx;

			for (auto& [joint_id, limb_ptr] : physical_animation_component->simulated_joints)
			{
				uint32 calculation_id = joint_id;
				physical_animation_component->simulated_joints_set.emplace(calculation_id);

				while (calculation_id != INVALID_JOINT)
				{
					const uint32 parent_id = skeleton->joints.at(calculation_id).parent_id;
					if (parent_id != INVALID_JOINT)
					{
						physical_animation_component->simulated_joints_set.emplace(parent_id);
						calculation_id = parent_id;
					}
					else
					{
						break;
					}
				}
			}

			const uint32 simulation_graph_size = static_cast<uint32>(physical_animation_component->simulated_joints_set.size() + 1u);

			physical_animation_component->local_joint_poses.reserve(simulation_graph_size);
			physical_animation_component->local_joint_poses_for_target_calculation.reserve(simulation_graph_size);

			physical_animation_component->local_joint_poses[INVALID_JOINT] = trs::identity;
			physical_animation_component->local_joint_poses_for_target_calculation[INVALID_JOINT] = trs::identity;
			physical_animation_component->local_joint_poses[0] = trs::identity;
			physical_animation_component->local_joint_poses_for_target_calculation[0] = trs::identity;

			physical_animation_component->try_to_apply_ragdoll_profile(idle_profile, RagdollProfileStrengthType::DEFAULT, true);
		}
	}

	void PhysicalAnimationSystem::on_pac_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		pacs_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}
}
