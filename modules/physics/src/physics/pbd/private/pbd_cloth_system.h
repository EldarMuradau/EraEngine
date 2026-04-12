#pragma once

#include <ecs/system.h>

#include <core/sync.h>

namespace era_engine
{
	class RendererHolderRootComponent;
}

namespace era_engine::physics
{
	class PBDClothSystem final : public System
	{
	public:
		PBDClothSystem(World* _world);

		void init() override;
		void update(float dt) override;

		void debug_draw(float dt);

		void process_added_clothes(float dt);

		void on_cloth_created(entt::registry& registry, entt::entity entity_handle);
		void on_cloth_released(entt::registry& registry, entt::entity entity_handle);

		ERA_VIRTUAL_REFLECT(System)

	private:
		std::vector<Entity::Handle> clothes_to_init;

		const RendererHolderRootComponent* renderer_holder_rc = nullptr;

		SpinLock sync;
	};
}