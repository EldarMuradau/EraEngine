#pragma once

#include "physics/physical_animation/dismemberment/ragdoll_dismemberment_component.h"
#include "physics/physical_animation/physical_animation_component.h"

#include <ecs/system.h>
#include <ecs/base_components/transform_component.h>

#include <animation/animation.h>
#include <animation/skeleton.h>

#include <core/sync.h>
#include <core/math.h>

#include <entt/entt.hpp>

namespace era_engine
{
	class RendererHolderRootComponent;
}

namespace era_engine::physics
{
	class CollisionsHolderRootComponent;

	class RagdollDismembermentSystem final : public System
	{
	public:
		RagdollDismembermentSystem(World* _world);

		void init() override;
		void update_ragdolls(float dt);

		void process_added_rdcs();

		void on_rdc_created(entt::registry& registry, entt::entity entity_handle);

		ERA_VIRTUAL_REFLECT(System)

	private:
		std::vector<Entity::Handle> rdcs_to_init;
		std::vector<RagdollDismembermentComponent*> active_ragdolls;

		SpinLock sync;

		ref<RagdollDismembermentProfile> dismemberment_profile;

		const CollisionsHolderRootComponent* collisions_holder_rc = nullptr;
		RendererHolderRootComponent* renderer_holder_rc = nullptr;

		entt::group<entt::owned_t<>,
			entt::get_t<TransformComponent,
			PhysicalAnimationComponent,
			RagdollDismembermentComponent,
			animation::SkeletonComponent>> ragdolls_group;
	};
}