#pragma once

#include "base_api.h"

#include <rttr/type>

namespace era_engine
{
	class World;

	enum class UpdateType : uint8_t
	{
		NORMAL = 0,
		FIXED
	};

	class ERA_BASE_API UpdateGroup
	{
	public:
		UpdateGroup(const char* _name, UpdateType _update_type, bool _main_thread = true);

		const char* name = nullptr;
		UpdateType update_type = UpdateType::NORMAL;
		bool main_thread = true;

		uint32 id = std::numeric_limits<uint32>::max();
	};

	ERA_BASE_API UpdateGroup* find_group(const std::string& name);

	class ERA_BASE_API System
	{
	public:
		System(World* _world);
		System(void* _world);

		virtual ~System();

		virtual void init();
		virtual void update(float dt);

		RTTR_ENABLE()

	protected:
		World* world = nullptr;

		friend class WorldSystemScheduler;
	};

	class ERA_BASE_API UpdatesHolder
	{
		UpdatesHolder() = delete;
	public:
		static std::unordered_map<std::string, UpdateGroup*> global_groups;
		static std::vector<std::string> update_order;
	};

}