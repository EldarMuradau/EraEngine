#pragma once

#include "base_api.h"

#include <rttr/type>

namespace era_engine
{
	class Engine;


	/**
	* Default module base class.
	*/
	class ERA_BASE_API IModule
	{
	public:
		enum class State : uint8_t
		{
			LOADING = 0,
			LOADED,
			UNLOADED,

			COUNT
		};

		enum class Type : uint8_t
		{
			NONE = 0,
			ENGINE,
			GAME,
			ASSET,
			THIRDPARTY,

			COUNT
		};

		IModule() = default;
		IModule(const char* _name) noexcept;

		virtual ~IModule() = default;

		virtual bool initialize(void* engine);
		virtual bool terminate();

		State get_state() const;
		const char* get_name() const;

		RTTR_ENABLE()

	protected:
		const char* name = nullptr;

		State state = State::LOADING;
	};
}

#define ERA_MODULE_REGISTRATION														\
	{																				\
		Engine * engine_ptr = static_cast<Engine*>(engine);							\
		World* world = get_world_by_name(World::GAMEPLAY_WORLD_NAME);				\
		WorldSystemScheduler* scheduler = world->get_system_scheduler();			\
		rttr::library module_lib(name);												\
		if (module_lib.load())														\
		{																			\
			scheduler->initialize_systems(module_lib.get_types());					\
		}																			\
		else																		\
		{																			\
			return false;															\
		}																			\
	}