#pragma once

#include "physics/ragdolls/ragdoll_component.h"

#include <ecs/system.h>
#include <ecs/base_components/transform_component.h>

#include <animation/animation.h>

#include <core/sync.h>

#include <entt/entt.hpp>

namespace era_engine::physics
{
	class RagdollSystem final : public System
	{
	public:
		RagdollSystem(World* _world);
		void init() override;
		void update(float dt) override;

		void update_normal(float dt);

		void process_added_ragdolls();

		void on_ragdoll_created(entt::registry& registry, entt::entity entity_handle);
		void on_ragdoll_removed(entt::registry& registry, entt::entity entity_handle);

		ERA_VIRTUAL_REFLECT(System)

	private:
		std::vector<Entity::Handle> ragdolls_to_init;
		SpinLock sync;

		entt::group<entt::owned_t<>, entt::get_t<TransformComponent,
			RagdollComponent,
			animation::SkeletonComponent>> ragdolls_group;
	};
}