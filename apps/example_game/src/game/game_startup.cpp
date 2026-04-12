#include "game/game_startup.h"
#include "game/physics/physics_initializer.h"

#include <engine/engine.h>

#include <base/module_loader.h>

#include <window/dx_window.h>

#include <core/string.h>

#include <physics_module.h>
#include <simple_motion_matching_module.h>

#include <ecs/world.h>
#include <ecs/world_system_scheduler.h>

namespace era_engine
{

	void GameStartup::start(int argc, char** argv)
	{
		Engine* engine = Engine::create_instance(argc, argv);

		auto initial_task = [&engine]() {
			ModuleLoader loader = ModuleLoader(engine);

			PhysicsModule* physics_module = new PhysicsModule();
			physics_module->initialize(engine);

			SimpleMotionMatchingModule* simple_mm_module = new SimpleMotionMatchingModule();
			simple_mm_module->initialize(engine);

			loader.add_module(physics_module);
			loader.add_module(simple_mm_module);

			ASSERT(loader.status());
		};

		dx_window* window = get_object<dx_window>();
		window->initialize(TEXT("  Example Game"), 1920, 1080);
		window->setIcon(get_asset_path("/resources/icons/Logo.ico"));
		window->setCustomWindowStyle();
		window->maximize();

		World* game_world = new World(World::GAMEPLAY_WORLD_NAME);
		game_world->get_system_scheduler()->set_fixed_update_rate(60.0f);
		game_world->init();

		game_world->add_tag("physics");
		game_world->add_tag("render");
		game_world->add_tag("base");
		game_world->add_tag("game");
		game_world->add_tag("motion_matching");
		game_world->add_tag("debug");

		PhysicsInitializer physics_initializer = PhysicsInitializer(game_world);
		physics_initializer.init();

		engine->run(initial_task);

		delete game_world;

		engine->terminate();
	}

}