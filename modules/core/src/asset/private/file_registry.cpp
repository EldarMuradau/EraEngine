#include "asset/file_registry.h"
#include "asset/asset.h"

#include "core/file_system.h"
#include "core/yaml.h"
#include "core/string.h"
#include "core/log.h"
#include "core/sync.h"

namespace era_engine
{
	typedef std::unordered_map<fs::path, AssetHandle> PathToHandle;
	typedef std::unordered_map<AssetHandle, fs::path> HandleToPath;

	static PathToHandle path_to_handle;
	static HandleToPath handle_to_path;

	static std::mutex file_registry_mutex;
	static const fs::path registry_path = fs::path(get_asset_path(L"/resources/files.yaml")).lexically_normal().make_preferred();

	static PathToHandle load_registry_from_disk()
	{
		PathToHandle loaded_registry;

		std::ifstream stream(registry_path);
		YAML::Node n = YAML::Load(stream);

		for (auto entry_node : n)
		{
			AssetHandle handle = 0;
			fs::path path;

			YAML_LOAD(entry_node, handle, "Handle");
			YAML_LOAD(entry_node, path, "Path");

			if (handle)
			{
				loaded_registry[path] = handle;
			}
		}

		return loaded_registry;
	}

	static void write_registry_to_disk()
	{
		YAML::Node out;

		for (const auto& [path, handle] : path_to_handle)
		{
			YAML::Node n;
			n["Handle"] = handle;
			n["Path"] = path;
			out.push_back(n);
		}

		fs::create_directories(registry_path.parent_path());
		std::ofstream fout(registry_path);
		fout << out;
	}

	static void read_directory(const fs::path& path, const PathToHandle& loaded_registry)
	{
		for (const auto& dir_entry : fs::directory_iterator(path))
		{
			const auto& path = dir_entry.path();
			fs::path normal_path = path.lexically_normal().make_preferred();

			if (dir_entry.is_directory())
			{
				read_directory(normal_path, loaded_registry);
			}
			else
			{
				auto it = loaded_registry.find(normal_path);

				// If already known, use the handle, otherwise generate one.
				AssetHandle handle = (it != loaded_registry.end()) ? it->second : AssetHandle::generate();

				path_to_handle.insert({ normal_path, handle });
				handle_to_path.insert({ handle, normal_path });
			}
		}
	}

	static void handle_asset_change(const FileSystemEvent& e)
	{
		if (!fs::is_directory(e.path) && e.path != registry_path)
		{
			fs::path normal_path = e.path.lexically_normal().make_preferred();

			{
				Lock lock{ file_registry_mutex };
				switch (e.change)
				{
				case FileSystemChange::Add:
				{
					LOG_MESSAGE("Asset '%ws' added", normal_path.c_str());

					ASSERT(path_to_handle.find(normal_path) == path_to_handle.end());

					AssetHandle handle = AssetHandle::generate();
					path_to_handle.insert({ normal_path, handle });
					handle_to_path.insert({ handle, normal_path });
				} break;

				case FileSystemChange::Delete:
				{
					LOG_MESSAGE("Asset '%ws' deleted", normal_path.c_str());

					auto it = path_to_handle.find(normal_path);

					if (it != path_to_handle.end())
					{
						AssetHandle handle = it->second;
						path_to_handle.erase(it);
						handle_to_path.erase(handle);
					}
					else
					{
						ASSERT(it != path_to_handle.end());
					}
				} break;

				case FileSystemChange::Modify:
				{
					LOG_MESSAGE("Asset '%ws' modified", normal_path.c_str());
				} break;

				case FileSystemChange::Rename:
				{
					fs::path old_path_normal = e.old_path.lexically_normal().make_preferred().c_str();
					LOG_MESSAGE("Asset renamed from '%ws' to '%ws'", old_path_normal, normal_path.c_str());

					auto old_it = path_to_handle.find(old_path_normal);

					ASSERT(old_it != path_to_handle.end());
					ASSERT(path_to_handle.find(normal_path) == path_to_handle.end());

					AssetHandle handle = old_it->second;
					path_to_handle.erase(old_it);
					path_to_handle.insert({ normal_path, handle });
					handle_to_path[handle] = normal_path;
				} break;
				}
			}

			// During runtime the registry is only written to in this function, so no need to protect the read with mutex.
			LOG_MESSAGE("Rewriting file registry");
			write_registry_to_disk();
		}
	}

	AssetHandle get_asset_handle_from_path(const fs::path& path)
	{
		const std::lock_guard<std::mutex> lock(file_registry_mutex);

		auto it = path_to_handle.find(path);
		if (it == path_to_handle.end())
		{
			return {};
		}
		return it->second;
	}

	fs::path get_path_from_asset_handle(AssetHandle handle)
	{
		const std::lock_guard<std::mutex> lock(file_registry_mutex);

		auto it = handle_to_path.find(handle);
		if (it == handle_to_path.end())
		{
			return {};
		}
		return it->second;
	}

	void initialize_file_registry()
	{
		auto loaded_registry = load_registry_from_disk();
		read_directory(get_asset_path(L"/resources/assets"), loaded_registry);
		write_registry_to_disk();

		observe_directory(get_asset_path(L"/resources/assets"), handle_asset_change);
	}

}