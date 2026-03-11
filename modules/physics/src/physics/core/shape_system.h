#pragma once

#include <ecs/system.h>

namespace era_engine::physics
{
	class ShapeSystem final : public System
	{
	public:
		ShapeSystem(World* _world);
		void init() override;
		void update(float dt) override;

		void process_skeleton_attachments(float dt) const;

		ERA_VIRTUAL_REFLECT(System)

	private:
		std::vector<Entity::Handle> shapes_to_init;
	};
}