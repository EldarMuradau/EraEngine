#pragma once

#include <ecs/system.h>
#include <ecs/entity.h>

#include <geometry/mesh.h>
#include <rendering/pbr_material.h>

namespace era_engine
{
	class GameInitSystem final : public System
	{
	public:
		GameInitSystem(World* _world);
		~GameInitSystem();

		void init() override;
		void update(float dt) override;

		ERA_VIRTUAL_REFLECT(System)

	private:
		Entity camera_entity;

		ref<multi_mesh> sphere_mesh;
		ref<pbr_material> sphere_render_material;
	};
}