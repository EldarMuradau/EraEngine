#include "physics/physical_animation/dismemberment/ragdoll_dismemberment_system.h"
#include "physics/core/physics.h"
#include "physics/core/physics_utils.h"
#include "physics/body_component.h"
#include "physics/joint.h"
#include "physics/core/physics_utils.h"
#include "physics/shape_utils.h"
#include "physics/collisions_holder_root_component.h"
#include "physics/shape_component.h"
#include "physics/physical_animation/ragdoll_profile.h"

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

		registration::class_<RagdollDismembermentSystem>("RagdollDismembermentSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("physics")))

			.method("update_ragdolls", &RagdollDismembermentSystem::update_ragdolls)
			(metadata("update_group", update_types::PHYSICS),
			 metadata("After", std::vector<std::string>{"PhysicalAnimationSystem::update_after_physics"}));
	}

	RagdollDismembermentSystem::RagdollDismembermentSystem(World* _world)
		: System(_world)
	{
	}

	void RagdollDismembermentSystem::init()
	{
		using namespace animation;
		entt::registry& registry = world->get_registry();
		registry.on_construct<RagdollDismembermentComponent>().connect<&RagdollDismembermentSystem::on_rdc_created>(this);

		ragdolls_group = world->group(components_group<TransformComponent, PhysicalAnimationComponent, RagdollDismembermentComponent, SkeletonComponent>);

		collisions_holder_rc = world->add_root_component<CollisionsHolderRootComponent>();
		renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();

		{
			dismemberment_profile = make_ref<RagdollDismembermentProfile>();

			{
				DismemberableLimbDetails& details = dismemberment_profile->limbs_details.emplace_back();
				details.limb_type = RagdollLimbType::HEAD;
				details.type = DismemberableLimbType::DISMEMBERABLE;
				details.max_health = 3.0f;
			}

			{
				DismemberableLimbDetails& details = dismemberment_profile->limbs_details.emplace_back();
				details.limb_type = RagdollLimbType::NECK;
				details.type = DismemberableLimbType::DISMEMBERABLE;
				details.max_health = 4.0f;
			}

			{
				DismemberableLimbDetails& details = dismemberment_profile->limbs_details.emplace_back();
				details.limb_type = RagdollLimbType::BODY_UPPER;
				details.type = DismemberableLimbType::ROOT;
				details.max_health = 1.0f;
			}

			{
				DismemberableLimbDetails& details = dismemberment_profile->limbs_details.emplace_back();
				details.limb_type = RagdollLimbType::BODY_MIDDLE;
				details.type = DismemberableLimbType::ROOT;
				details.max_health = 1.0f;
			}

			{
				DismemberableLimbDetails& details = dismemberment_profile->limbs_details.emplace_back();
				details.limb_type = RagdollLimbType::BODY_LOWER;
				details.type = DismemberableLimbType::ROOT;
				details.max_health = 1.0f;
			}

			{
				DismemberableLimbDetails& details = dismemberment_profile->limbs_details.emplace_back();
				details.limb_type = RagdollLimbType::CLAVICLE;
				details.type = DismemberableLimbType::ROOT;
				details.max_health = 1.0f;
			}

			{
				DismemberableLimbDetails& details = dismemberment_profile->limbs_details.emplace_back();
				details.limb_type = RagdollLimbType::ARM;
				details.type = DismemberableLimbType::DISMEMBERABLE;
				details.max_health = 3.0f;
			}

			{
				DismemberableLimbDetails& details = dismemberment_profile->limbs_details.emplace_back();
				details.limb_type = RagdollLimbType::FOREARM;
				details.type = DismemberableLimbType::DISMEMBERABLE;
				details.max_health = 2.0f;
			}

			{
				DismemberableLimbDetails& details = dismemberment_profile->limbs_details.emplace_back();
				details.limb_type = RagdollLimbType::HAND;
				details.type = DismemberableLimbType::ROOT;
				details.max_health = 1.0f;
			}

			{
				DismemberableLimbDetails& details = dismemberment_profile->limbs_details.emplace_back();
				details.limb_type = RagdollLimbType::LEG;
				details.type = DismemberableLimbType::DISMEMBERABLE;
				details.max_health = 3.0f;
			}

			{
				DismemberableLimbDetails& details = dismemberment_profile->limbs_details.emplace_back();
				details.limb_type = RagdollLimbType::CALF;
				details.type = DismemberableLimbType::DISMEMBERABLE;
				details.max_health = 2.0f;
			}

			{
				DismemberableLimbDetails& details = dismemberment_profile->limbs_details.emplace_back();
				details.limb_type = RagdollLimbType::FOOT;
				details.type = DismemberableLimbType::ROOT;
				details.max_health = 1.0f;
			}
		}

	}

	void RagdollDismembermentSystem::update_ragdolls(float dt)
	{
		ZoneScopedN("RagdollDismembermentSystem::update_ragdolls");

		process_added_rdcs();

		for (auto&& [entity_handle, transform_component, physical_animation_component, dismemberment_component, skeleton] : ragdolls_group.each())
		{
			if (!physical_animation_component.loaded ||
				!dismemberment_component.loaded)
			{
				continue;
			}

			const ref<RagdollDismembermentProfile>& proflie = dismemberment_component.dismemberment_profile;

			std::vector<Entity> dismembered_limbs;

			for (EntityPtr limb_ptr : physical_animation_component.limbs)
			{
				Entity limb = limb_ptr.get();

				RagdollDismembermentLimbComponent* dismemberment_limb_component = limb.get_component<RagdollDismembermentLimbComponent>();
				PhysicalAnimationLimbComponent* limb_component = limb.get_component<PhysicalAnimationLimbComponent>();

				if (dismemberment_component.affected_by_collisions &&
					limb_component->collision.is_colliding)
				{
					//const float total_applied_impulse = length(limb_component->collision.impulse);
					//const float applied_force = total_applied_impulse / dt;
					dismemberment_limb_component->apply_damage(/*applied_force*/ 1.0f);
				}

				if (dismemberment_limb_component->is_pending())
				{
					dismemberment_limb_component->state = DismemberState::DISMEMBERED;
					limb_component->force_switch_state(PhysicalLimbStateType::DISMEMBERED);

					if (dismemberment_component.on_dismember != nullptr)
					{
						dismemberment_component.on_dismember(dismemberment_limb_component);
					}

					dismembered_limbs.push_back(limb);
				}
			}

			for (Entity limb : dismembered_limbs)
			{
				PhysicalAnimationLimbComponent* limb_component = limb.get_component<PhysicalAnimationLimbComponent>();

				const PhysicsLimbChain* chain = physical_animation_component.get_chain_by_joint_id(limb_component->joint_id);
				if (chain == nullptr)
				{
					ASSERT(chain != nullptr);
					continue;
				}

				bool is_next_limb = false;

				for (EntityPtr limb_ptr : chain->connected_limbs)
				{
					Entity chain_limb = limb_ptr.get();

					if (chain_limb == limb)
					{
						is_next_limb = true;
						continue;
					}

					if (!is_next_limb)
					{
						continue;
					}

					RagdollDismembermentLimbComponent* next_dismemberment_limb_component = chain_limb.get_component<RagdollDismembermentLimbComponent>();
					if (next_dismemberment_limb_component->get_current_state() == DismemberState::DISMEMBERED)
					{
						continue;
					}

					next_dismemberment_limb_component->state = DismemberState::PARTIAL_DISMEMBERED;

					PhysicalAnimationLimbComponent* next_limb_component = chain_limb.get_component<PhysicalAnimationLimbComponent>();
					next_limb_component->force_switch_state(PhysicalLimbStateType::PARTIAL_DISMEMBERED);
				}
			}
		}
	}

	void RagdollDismembermentSystem::process_added_rdcs()
	{
		using namespace physx;
		using namespace animation;

		ScopedSpinLock _lock{ sync };

		for (auto iter = rdcs_to_init.begin(); iter != rdcs_to_init.end();)
		{
			Entity::Handle entity_handle = *iter;
			Entity entity = world->get_entity(entity_handle);

			const PhysicalAnimationComponent* physical_animation_component = entity.get_component_if_exists<PhysicalAnimationComponent>();
			if (physical_animation_component == nullptr ||
				!physical_animation_component->loaded)
			{
				++iter;
				continue;
			}

			const SkeletonComponent* skeleton_component = entity.get_component<SkeletonComponent>();

			const ref<Skeleton>& skeleton = skeleton_component->skeleton;
			if (skeleton == nullptr)
			{
				++iter;
				continue;
			}

			RagdollDismembermentComponent* dismemberment_component = entity.get_component<RagdollDismembermentComponent>();

			{
				dismemberment_component->dismemberment_profile = dismemberment_profile;
			}

			const ref<RagdollDismembermentProfile>& proflie = dismemberment_component->dismemberment_profile;

			bool is_loaded = true;

			for (EntityPtr limb_ptr : physical_animation_component->limbs)
			{
				Entity limb = limb_ptr.get();

				PhysicalAnimationLimbComponent* limb_component = limb.get_component<PhysicalAnimationLimbComponent>();

				const DismemberableLimbDetails* limb_details = proflie->get_details_by_limb_type(limb_component->type);
				if (limb_details == nullptr)
				{
					is_loaded = false;
					ASSERT(limb_details != nullptr);
					break;
				}

				RagdollDismembermentLimbComponent* dismemberment_limb_component = limb.add_component<RagdollDismembermentLimbComponent>();
				dismemberment_limb_component->ragdoll_ptr = entity;
				dismemberment_limb_component->can_be_dismembered = limb_details->type == DismemberableLimbType::DISMEMBERABLE;
				dismemberment_limb_component->current_health = limb_details->max_health;
			}

			if(is_loaded)
			{
				dismemberment_component->loaded = true;
				iter = rdcs_to_init.erase(iter);
			}
			else
			{
				++iter;
			}
		}
	}

	void RagdollDismembermentSystem::on_rdc_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		rdcs_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}
}
