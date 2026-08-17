#pragma once

#include <ecs/system.h>

#include <core/sync.h>
#include <core/math.h>

#include <rendering/pbr_material.h>

namespace era_engine
{
	struct SubmeshAsset;
}

namespace era_engine::physics
{
	class NvMesh;
	class DestructibleComponent;

	class DestructionSystem final : public System
	{
	public:
		DestructionSystem(World* _world);

		void init() override;
		void update(float dt) override;

		void process_added_destructibles(float dt);

		void on_destructible_created(entt::registry& registry, entt::entity entity_handle);

	private:
		bool init_fracture_based_entity(Entity entity, DestructibleComponent* destructibe_component) const;

		EntityPtr build_chunk(Entity parent,
			const trs& world_transform,
			const ref<pbr_material>& material,
			const std::pair<ref<SubmeshAsset>, ref<NvMesh>>& mesh,
			float density,
			float& radius) const;

		std::vector<EntityPtr> build_chunks(Entity parent,
			const trs& world_transform,
			const ref<pbr_material>& material,
			const std::vector<std::pair<ref<SubmeshAsset>, ref<NvMesh>>>& meshes,
			float density,
			std::vector<float>& radiuses) const;

		bool connect_touching_chunks(Entity parent, DestructibleComponent* destructibe_component) const;

		void connect_with_neighbours(DestructibleComponent* parent_destructibe_component, Entity chunk) const;

		ERA_VIRTUAL_REFLECT(System)

	private:
		std::vector<Entity::Handle> destructibles_to_init;

		SpinLock sync;
	};
}