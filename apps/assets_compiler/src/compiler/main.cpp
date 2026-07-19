// Copyright (c) 2023-present Eldar Muradov. All rights reserved.
#include <fstream>
#include <iostream>

#include <clara/clapa.hpp>

#include <asset/bin.h>
#include <asset/model_asset.h>

#include <core/log.h>
#include <core/string.h>

int main(int argc, char** argv)
{
	using namespace era_engine;
	using namespace clara;

	try
	{
		fs::path path;
		bool verbose = false;

		Parser cli;
		cli += Opt(verbose, "verbose")["-v"]["--verbose"]("Enable verbose logging");
		cli += Opt(path, "path")["-p"]["--path"]("Path to asset");

		auto result = cli.parse(Args(argc, argv));
		if (!result)
		{
			std::cerr << "Error in command line: " << result.errorMessage() << std::endl;
		}

		path = get_full_path(path);

		if (!fs::exists(path))
		{
			std::cerr << "Could not find file '" << path << "'.\n";
			return EXIT_FAILURE;
		}

		std::string extension = path.extension().string();

		fs::path cached_filename = path;
		cached_filename.replace_extension("." + std::to_string(MESH_FLAG_DEFAULT) + ".cache.bin");
		fs::path cache_filepath = L"asset_cache" / cached_filename;

		if (fs::exists(cache_filepath))
		{
			auto last_cache_write_time = fs::last_write_time(cache_filepath);
			auto last_original_write_time = fs::last_write_time(path);

			if (last_cache_write_time > last_original_write_time)
			{
				std::cout << "Asset is already compiled! \n";
				return EXIT_SUCCESS;
			}
		}

		std::cout << "Preprocessing asset '" << path << "' for faster loading next time.\n";

		ModelAsset result_mesh;

		std::transform(extension.begin(), extension.end(), extension.begin(),
			[](char c) { return std::tolower(c); });
		if (extension == ".fbx")
		{
			result_mesh = loadFBX(path, MESH_FLAG_DEFAULT);
		}
		else if (extension == ".obj")
		{
			result_mesh = loadOBJ(path, MESH_FLAG_DEFAULT);
		}

		fs::create_directories(cache_filepath.parent_path());
		writeBIN(result_mesh, cache_filepath);
	}
	catch (const std::exception& ex)
	{
		std::cerr << ex.what() << "\n";

		std::ofstream output("logs/compiler_error_log.txt");
		output << "Runtime Error>" << ex.what() << std::endl;
		output.close();

		std::this_thread::sleep_for(std::chrono::duration<float>(5000.0f));
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}