#pragma once

#include "asset/io.h"
#include "asset/file_registry.h"

namespace era_engine
{
	template<class T, CompatibleAssetType<T>>
	inline ref<T> GameAssetsProvider::load_game_asset_from_file(const fs::path& path, bool load_from_cache/* = true*/, JobHandle parent_job /*= {}*/, uint32 flags /*= 0*/)
	{
		fs::path final_path = path.parent_path();
		final_path.append(path.filename().string() + AssetExtension<T>::get_asset_type());
		final_path = final_path.lexically_normal().make_preferred();

		if(load_from_cache)
		{
			ref<GameAsset> cached_asset = GameAssetCache::try_get_asset_from_cache(final_path, flags);
			if (cached_asset != nullptr)
			{
				return std::dynamic_pointer_cast<T>(cached_asset);
			}
		}

		ref<T> asset = make_ref<T>();
		asset->handle = getAssetHandleFromPath(final_path);
		asset->flags = flags;

		GameAssetCache::try_add_asset_to_cache(asset, final_path, flags);

		struct AssetLoadingData
		{
			ref<T> asset = nullptr;
			fs::path path;
		};

		AssetLoadingData* data = new AssetLoadingData{ asset, final_path };

		DefaultJob job_object;

		job_object.data.user_data = data;
		job_object.data.data_size = sizeof(AssetLoadingData);

		job_object.job_func = [](DefaultJob::Data& default_data)
			{
				AssetLoadingData* data = reinterpret_cast<AssetLoadingData*>(default_data.user_data);

				std::ifstream is(data->path, std::ios_base::binary);

				if (!is.is_open())
				{
					data->asset->load_state = AssetLoadState::UNLOADED;
					return;
				}

				if (!data->asset->deserialize(is))
				{
					data->asset->load_state = AssetLoadState::UNLOADED;
					is.close();
					return;
				}

				data->asset->handle = getAssetHandleFromPath(data->path);
				data->asset->load_state = AssetLoadState::LOADED;

				is.close();
			};

		JobHandle job = schedule_low_priority_job(job_object, parent_job);
		job.submit_now();

		asset->load_job = job;

		return asset;
	}

	template<class T, CompatibleAssetType<T>>
	inline JobHandle GameAssetsProvider::save_game_asset_to_file_async(const fs::path& path, const T* asset, JobHandle parent_job/* = {}*/, bool append_ext/* = true*/)
	{
		if (asset == nullptr)
		{
			return {};
		}

		fs::path final_path = path.parent_path();
		std::string filename = path.filename().string();
		if (append_ext)
		{
			filename += AssetExtension<T>::get_asset_type();
		}
		final_path.append(filename);
		final_path = final_path.lexically_normal().make_preferred();

		struct AssetLoadingData
		{
			const T* asset = nullptr;
			fs::path path;
		};

		AssetLoadingData* data = new AssetLoadingData{ asset, final_path };

		DefaultJob job_object;

		job_object.data.user_data = data;
		job_object.data.data_size = sizeof(AssetLoadingData);

		job_object.job_func = [](DefaultJob::Data& default_data)
			{
				AssetLoadingData* data = reinterpret_cast<AssetLoadingData*>(default_data.user_data);

				if (!std::filesystem::exists(data->path.parent_path()))
				{
					if (!std::filesystem::create_directories(data->path.parent_path()))
					{
						return;
					}
				}

				std::ofstream of(data->path, std::ios_base::binary);

				if (!of.is_open())
				{
					return;
				}

				if (!data->asset->serialize(of))
				{
					of.close();
					return;
				}

				of.close();
			};

		JobHandle job = schedule_low_priority_job(job_object, parent_job);
		job.submit_now();

		return job;
	}
}