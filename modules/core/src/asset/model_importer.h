#pragma once

#include "core_api.h"

namespace era_engine
{
	struct ModelAsset;

	ERA_CORE_API ModelAsset universal_load_model(const fs::path& path, uint32 flags);
}