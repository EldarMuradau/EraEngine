#pragma once

#include "ecs/update_groups.h"

namespace era_engine::update_types
{
	void register_default_order()
	{
		UpdatesHolder::update_order.push_back(std::string(INPUT.name));

		UpdatesHolder::update_order.push_back(std::string(BEGIN.name));

		UpdatesHolder::update_order.push_back(std::string(GAMEPLAY_BEFORE_PHYSICS.name));
		UpdatesHolder::update_order.push_back(std::string(GAMEPLAY_BEFORE_PHYSICS_CONCURRENT.name));

		UpdatesHolder::update_order.push_back(std::string(BEFORE_PHYSICS.name));
		UpdatesHolder::update_order.push_back(std::string(PHYSICS.name));
		UpdatesHolder::update_order.push_back(std::string(AFTER_PHYSICS.name));

		UpdatesHolder::update_order.push_back(std::string(GAMEPLAY_AFTER_PHYSICS.name));
		UpdatesHolder::update_order.push_back(std::string(GAMEPLAY_AFTER_PHYSICS_CONCURRENT.name));

		UpdatesHolder::update_order.push_back(std::string(GAMEPLAY_NORMAL.name));
		UpdatesHolder::update_order.push_back(std::string(GAMEPLAY_NORMAL_CONCURRENT.name));

		UpdatesHolder::update_order.push_back(std::string(BEFORE_RENDER.name));
		UpdatesHolder::update_order.push_back(std::string(RENDER.name));
		UpdatesHolder::update_order.push_back(std::string(AFTER_RENDER.name));

		UpdatesHolder::update_order.push_back(std::string(END.name));
	}
}