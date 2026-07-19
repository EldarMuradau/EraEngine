// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core_api.h"

namespace era_engine
{
	struct AssetHandle;

	ERA_CORE_API AssetHandle get_asset_handle_from_path(const fs::path& path);
	ERA_CORE_API fs::path get_path_from_asset_handle(AssetHandle handle);

	ERA_CORE_API void initialize_file_registry();
}