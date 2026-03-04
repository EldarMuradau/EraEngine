#include "physics/ragdolls/ragdoll_system.h"
#include "physics/core/physics.h"
#include "physics/body_component.h"
#include "physics/joint.h"
#include "physics/core/physics_utils.h"
#include "physics/shape_utils.h"
#include "physics/ragdolls/ragdoll_builder.h"
#include "physics/shape_component.h"

#include <core/cpu_profiling.h>

#include <ecs/base_components/transform_component.h>
#include <ecs/update_groups.h>

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<RagdollSystem>("RagdollSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("physics")))

			.method("update", &RagdollSystem::update)
			(metadata("update_group", update_types::PHYSICS),
			 metadata("After", std::vector<std::string>{"PhysicsSystem::update"}))

			.method("update_normal", &RagdollSystem::update_normal)
			(metadata("update_group", update_types::GAMEPLAY_NORMAL));
	}

	RagdollSystem::RagdollSystem(World* _world)
		: System(_world)
	{
	}

	void RagdollSystem::init()
	{
		entt::registry& registry = world->get_registry();
		registry.on_construct<RagdollComponent>().connect<&RagdollSystem::on_ragdoll_created>(this);

		ragdolls_group = world->group(components_group<TransformComponent, RagdollComponent, animation::SkeletonComponent>);
	}

	void RagdollSystem::update(float dt)
	{
		ZoneScopedN("RagdollSystem::update");

		process_added_ragdolls();

		for (auto&& [entity_handle, transform_component, ragdoll_component, skeleton_component] : ragdolls_group.each())
		{
			if (ragdoll_component.loaded)
			{
				continue;
			}
			bool is_ready_for_simulation = true;
			for (const EntityPtr& limb_ptr : ragdoll_component.limbs)
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

			ragdoll_component.loaded = is_ready_for_simulation;
		}

		for (auto [entity_handle, transform_component, ragdoll_component] : world->group(components_group<TransformComponent, RagdollComponent>).each())
		{
			if (!ragdoll_component.loaded)
			{
				continue;
			}

			if (ragdoll_component.simulated.is_changed())
			{
				for (const EntityPtr& limb_ptr : ragdoll_component.limbs)
				{
					Entity limb = limb_ptr.get();
					limb.get_component<RagdollLimbComponent>()->simulated = ragdoll_component.simulated;
				}
				ragdoll_component.simulated.sync_changes();
			}
			ragdoll_component.elapsed_blend_time = 0.0f;
			ragdoll_component.reached_physics_pose = false;
		}

		for (auto [entity_handle, transform_component, ragdoll_limb_component] : world->group(components_group<TransformComponent, RagdollLimbComponent>).each())
		{
			if (ragdoll_limb_component.simulated.is_changed())
			{
				Entity limb = world->get_entity(entity_handle);
				limb.get_component<DynamicBodyComponent>()->simulated = ragdoll_limb_component.simulated;
				ragdoll_limb_component.simulated.sync_changes();
			}

			ragdoll_limb_component.prev_physics_pose = ragdoll_limb_component.physics_pose;
			ragdoll_limb_component.physics_pose = transform_component.get_local_transform();
		}
	}

	void RagdollSystem::update_normal(float dt)
	{
		using namespace animation;

		ZoneScopedN("RagdollSystem::update_normal");

		for (auto&& [entity_handle, transform_component, ragdoll_component, skeleton_component] : ragdolls_group.each())
		{
			if (!ragdoll_component.simulated || !ragdoll_component.loaded)
			{
				continue;
			}

			Entity ragdoll = world->get_entity(entity_handle);

			ref<Skeleton> skeleton = skeleton_component.skeleton;
			if (skeleton == nullptr)
			{
				continue;
			}
			
			{
				const float fixed_update_dt = world->get_fixed_update_dt();

				const uint32 root_id = 0;
				ragdoll_component.local_joint_poses[root_id] = SkeletonUtils::get_object_space_joint_transform(skeleton.get(), root_id);

				auto simulated_joints_end = ragdoll_component.simulated_joints.end();

				if (!ragdoll_component.reached_physics_pose)
				{
					ragdoll_component.elapsed_blend_time += dt;
				}

				const float blend_value = ragdoll_component.reached_physics_pose ? 1.0f : std::min(ragdoll_component.elapsed_blend_time / fixed_update_dt, 1.0f);
				if (ragdoll_component.elapsed_blend_time >= fixed_update_dt)
				{
					ragdoll_component.reached_physics_pose = true;
				}

				for (const uint32 simulation_joint : ragdoll_component.simulated_joints_set)
				{
					const uint32 parent_id = skeleton->joints.at(simulation_joint).parent_id;
					const trs& parent_local = ragdoll_component.local_joint_poses[parent_id];

					trs inverse_parent_local = invert(parent_local);

					trs limb_current_pose = skeleton->get_joint_transform(simulation_joint);
					limb_current_pose.scale = vec3(1.0f);

					auto limb_iter = ragdoll_component.simulated_joints.find(simulation_joint);
					if (limb_iter == ragdoll_component.simulated_joints.end())
					{
						const trs calculated_transform = parent_local * limb_current_pose;
						ragdoll_component.local_joint_poses[simulation_joint] = calculated_transform;
					}
					else
					{
						Entity limb = limb_iter->second.get();
						RagdollLimbComponent* ragdoll_limb = limb.get_component<RagdollLimbComponent>();

						trs new_transform = inverse_parent_local * ragdoll_limb->physics_pose;

						const trs prev_transform = inverse_parent_local * ragdoll_limb->prev_physics_pose;

						if (!fuzzy_equals(prev_transform, new_transform))
						{
							new_transform.position = lerp(prev_transform.position,
								new_transform.position,
								blend_value);

							new_transform.rotation = slerp(prev_transform.rotation,
								new_transform.rotation,
								blend_value);
						}

						new_transform.rotation = normalize(new_transform.rotation);

						skeleton->set_joint_transform(new_transform, simulation_joint);

						trs new_local_child_transform = parent_local * new_transform;
						new_local_child_transform.rotation = normalize(new_local_child_transform.rotation);

						ragdoll_component.local_joint_poses[simulation_joint] = new_local_child_transform;
					}
				}
			}
		}
	}

	void RagdollSystem::process_added_ragdolls()
	{
		using namespace physx;
		using namespace animation;

		ScopedSpinLock _lock{ sync };

		for (Entity::Handle entity_handle : std::exchange(ragdolls_to_init, {}))
		{
			Entity entity = world->get_entity(entity_handle);

			const SkeletonComponent* skeleton_component = entity.get_component<SkeletonComponent>();

			const ref<Skeleton> skeleton = skeleton_component->skeleton;
			if (skeleton == nullptr)
			{
				continue;
			}

			RagdollBuilderUtils::build_simulated_body(entity);

			RagdollComponent* ragdoll_component = entity.get_component<RagdollComponent>();

			for (const EntityPtr& limb_ptr : ragdoll_component->limbs)
			{
				const RagdollLimbComponent* limb_component = limb_ptr.get().get_component<RagdollLimbComponent>();
				ragdoll_component->simulated_joints.emplace(limb_component->joint_id, limb_ptr);
			}

			ragdoll_component->root_joint_id = ragdoll_component->joint_init_ids.attachment_idx;

			for (auto& [joint_id, limb_ptr] : ragdoll_component->simulated_joints)
			{
				uint32 calculation_id = joint_id;
				ragdoll_component->simulated_joints_set.emplace(calculation_id);

				while (calculation_id != INVALID_JOINT)
				{
					const uint32 parent_id = skeleton->joints.at(calculation_id).parent_id;
					if (parent_id != INVALID_JOINT)
					{
						ragdoll_component->simulated_joints_set.emplace(parent_id);
						calculation_id = parent_id;
					}
					else
					{
						break;
					}
				}
			}

			const uint32 simulation_graph_size = static_cast<uint32>(ragdoll_component->simulated_joints_set.size() + 1u);

			ragdoll_component->local_joint_poses.reserve(simulation_graph_size);
			ragdoll_component->local_joint_poses[0] = trs::identity;
			ragdoll_component->local_joint_poses[INVALID_JOINT] = trs::identity;
		}
	}

	void RagdollSystem::on_ragdoll_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		ragdolls_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}

}
