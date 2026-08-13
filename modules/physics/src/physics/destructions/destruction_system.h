#pragma once

#include <ecs/system.h>

#include <core/sync.h>

#include "rendering/pbr_material.h"

namespace era_engine
{
	struct SubmeshAsset;
}

struct trs;

namespace era_engine::physics
{
	class NvMesh;

	class DestructionSystem final : public System
	{
	public:
		DestructionSystem(World* _world);

		void init() override;
		void update(float dt) override;

		void process_added_destructibles(float dt);

		void on_destructible_created(entt::registry& registry, entt::entity entity_handle);

		Entity build_chunk(Entity parent,
			const trs& world_transform,
			const ref<pbr_material>& material,
			const std::pair<ref<SubmeshAsset>, ref<NvMesh>>& mesh,
			float density,
			float& radius);

		std::vector<Entity> build_chunks(Entity parent,
			const trs& world_transform,
			const ref<pbr_material>& material,
			const std::vector<std::pair<ref<SubmeshAsset>, ref<NvMesh>>>& meshes,
			float density,
			std::vector<float>& radiuses);

		ERA_VIRTUAL_REFLECT(System)

	private:
		std::vector<Entity::Handle> destructibles_to_init;

		SpinLock sync;
	};
}