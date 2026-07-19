#pragma once 

#include "core_api.h"

namespace era_engine
{
	struct ModelAsset;

	ERA_CORE_API void write_bin(const ModelAsset& asset, const fs::path& path);

	ERA_CORE_API ModelAsset load_fbx(const fs::path& path, uint32 flags);
	ERA_CORE_API ModelAsset load_obj(const fs::path& path, uint32 flags);
	ERA_CORE_API ModelAsset load_bin(const fs::path& path);
}