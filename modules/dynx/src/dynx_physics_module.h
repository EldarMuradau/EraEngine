#pragma once

#include "dynx_api.h"

#include <base/module.h>

namespace era_engine
{
	class ERA_DYNX_API DynXPhysicsModule : public IModule
	{
	public:
		DynXPhysicsModule() noexcept;
		~DynXPhysicsModule() override;

		bool initialize(void* engine) override;
		bool terminate() override;

		RTTR_ENABLE(IModule)
	};
}