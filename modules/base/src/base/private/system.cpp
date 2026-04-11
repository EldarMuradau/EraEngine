#include "base/system.h"

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<System>("System")
			.constructor<void*>()(policy::ctor::as_raw_ptr);
	}

	std::unordered_map<std::string, UpdateGroup*> UpdatesHolder::global_groups;
	std::vector<std::string> UpdatesHolder::update_order;

	static std::mutex group_mutex;
	static uint32 last_group_id = 0;

	UpdateGroup* find_group(const std::string& name)
	{
		std::lock_guard<std::mutex> lock{ group_mutex };

		auto iter = UpdatesHolder::global_groups.find(name);
		if (iter == UpdatesHolder::global_groups.end())
		{
			return nullptr;
		}
		return iter->second;
	}

	UpdateGroup::UpdateGroup(const char* _name, UpdateType _update_type, bool _main_thread/* = true*/)
		: name(_name), update_type(_update_type), main_thread(_main_thread)
	{
		std::lock_guard<std::mutex> lock{group_mutex};
		UpdatesHolder::global_groups.emplace(std::string(_name), this);
		id = last_group_id++;
	}

	System::System(World* _world)
		: world(_world)
	{
	}

	System::System(void* _world)
		: world(static_cast<World*>(_world))
	{
	}

	System::~System()
	{
	}

	void System::init()
	{
	}

	void System::update(float dt)
	{
	}
}