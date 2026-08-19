#pragma once

#include <ecs/system.h>

#include <core/sync.h>

namespace era_engine::physics
{
	class JointsSystem final : public System
	{
	public:
		JointsSystem(World* _world);
		void init() override;
		void update(float dt) override;

		void sync_physics_to_component_changes();
		void process_added_joints();

		void on_fixed_joint_created(entt::registry& registry, entt::entity entity_handle);
		void on_distance_joint_created(entt::registry& registry, entt::entity entity_handle);
		void on_d6_joint_created(entt::registry& registry, entt::entity entity_handle);
		void on_spherical_joint_created(entt::registry& registry, entt::entity entity_handle);
		void on_revolute_joint_created(entt::registry& registry, entt::entity entity_handle);

		void on_fixed_joint_removed(entt::registry& registry, entt::entity entity_handle);
		void on_distance_joint_removed(entt::registry& registry, entt::entity entity_handle);
		void on_d6_joint_removed(entt::registry& registry, entt::entity entity_handle);
		void on_spherical_joint_removed(entt::registry& registry, entt::entity entity_handle);
		void on_revolute_joint_removed(entt::registry& registry, entt::entity entity_handle);

		ERA_VIRTUAL_REFLECT(System)

	private:
		std::vector<Entity::Handle> d6_joints_to_init;
		std::vector<Entity::Handle> fixed_joints_to_init;
		std::vector<Entity::Handle> distance_joints_to_init;
		std::vector<Entity::Handle> revolute_joints_to_init;
		std::vector<Entity::Handle> spherical_joints_to_init;

		SpinLock sync;
	};
}