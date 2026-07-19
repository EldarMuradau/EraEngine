// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "asset/model_asset.h"
#include "asset/bin.h"
#include "core/log.h"

#include "rendering/pbr_material.h"

namespace era_engine
{
	ModelAsset import_3d_model_from_file(const fs::path& path, uint32 mesh_flags)
	{
		if (!fs::exists(path))
		{
			LOG_WARNING("Could not find file '%ws'", path.c_str());
			std::cerr << "Could not find file '" << path << "'.\n";
			return {};
		}

		std::string extension = path.extension().string();

		fs::path cached_filename = path;
		cached_filename.replace_extension("." + std::to_string(mesh_flags) + ".cache.bin");
		fs::path cache_filepath = L"asset_cache" / cached_filename;

		if (fs::exists(cache_filepath))
		{
			auto last_cache_write_time = fs::last_write_time(cache_filepath);
			auto last_original_write_time = fs::last_write_time(path);

			if (last_cache_write_time > last_original_write_time)
			{
				return load_bin(cache_filepath);
			}
		}

		LOG_MESSAGE("Preprocessing asset '%ws' for faster loading next time", path.c_str());
		std::cout << "Preprocessing asset '" << path << "' for faster loading next time.";
#ifdef _DEBUG
		std::cout << " Consider running in a release build the first time.";
#endif
		std::cout << '\n';

		ModelAsset result;

		std::transform(extension.begin(), extension.end(), extension.begin(),
			[](char c) { return std::tolower(c); });
		if (extension == ".fbx")
		{
			result = load_fbx(path, mesh_flags);
		}
		else if (extension == ".obj")
		{
			result = load_obj(path, mesh_flags);
		}

		fs::create_directories(cache_filepath.parent_path());
		write_bin(result, cache_filepath);

		return result;
	}
}