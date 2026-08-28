#pragma once

#include "physics/body_component.h"

#include <ecs/base_components/transform_component.h>
#include <ecs/system.h>

#include <core/sync.h>

#include <entt/entt.hpp>

namespace era_engine::physics
{
	class CharacterControllerHitReportCallback;

	class PhysicsSystem final : public System
	{
	public:
		PhysicsSystem(World* _world);
		void init() override;
		void update(float dt) override;

		void clear_pending_collisions(float);

		void sync_physics_to_component_changes(float dt);
		void sync_component_to_physics(float dt);

		void process_added_components(float);

		void process_added_bodies();
		void process_added_ccts();

		void on_dynamic_body_created(entt::registry& registry, entt::entity entity_handle);
		void on_static_body_created(entt::registry& registry, entt::entity entity_handle);
		void on_aggregate_created(entt::registry& registry, entt::entity entity_handle);
		void on_cct_created(entt::registry& registry, entt::entity entity_handle);

		void on_body_removed(entt::registry& registry, entt::entity entity_handle);
		void on_cct_removed(entt::registry& registry, entt::entity entity_handle);
		void on_aggregate_removed(entt::registry& registry, entt::entity entity_handle);

		ERA_VIRTUAL_REFLECT(System)

	private:
		std::vector<Entity::Handle> dynamics_to_init;
		std::vector<Entity::Handle> statics_to_init;
		std::vector<Entity::Handle> aggregates_to_init;
		std::vector<Entity::Handle> ccts_to_init;

		entt::group<entt::owned_t<>, entt::get_t<TransformComponent, DynamicBodyComponent>> dynamic_body_group;

		ref<CharacterControllerHitReportCallback> cct_hit_report_callback = nullptr;

		SpinLock sys_sync;
	};
}