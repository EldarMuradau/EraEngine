#pragma once

#include "core_api.h"

#include "asset/asset.h"

#include "core/job_system.h"

#include <fstream>

namespace era_engine
{
	class EntireFile;

	template <typename T>
	class AssetExtension
	{
	public:
		static std::string get_asset_type()
		{
			return T::get_asset_type_impl();
		}
	};

	class ERA_CORE_API GameAsset
	{
	public:
		GameAsset() = default;
		virtual ~GameAsset() = default;

		virtual bool serialize(std::ostream& os) const = 0;
		virtual bool deserialize(std::istream& is) = 0;

		AssetHandle handle;
		uint32 flags = 0;

		std::atomic<AssetLoadState> load_state = AssetLoadState::UNLOADED;
		JobHandle load_job;
	};

	class ERA_CORE_API GameAssetCache
	{
	public:
		GameAssetCache() = delete;

		static ref<GameAsset> try_get_asset_from_cache(const fs::path& filename, uint32 flags);
		static void try_add_asset_to_cache(ref<GameAsset> asset, const fs::path& filename, uint32 flags);
	};


	template <typename T>
	using CompatibleAssetType = std::enable_if_t<std::is_base_of_v<GameAsset, T>, bool>;

	class ERA_CORE_API GameAssetsProvider final
	{
	public:
		GameAssetsProvider() = default;

		template<class T, CompatibleAssetType<T> = true>
		ref<T> load_game_asset_from_file(const fs::path& path, bool load_from_cache = true, JobHandle parent_job = {}, uint32 flags = 0);

		template<class T, CompatibleAssetType<T> = true>
		JobHandle save_game_asset_to_file_async(const fs::path& path, const T* asset, JobHandle parent_job = {}, bool append_ext = true);
	};
}

#include "asset/private/game_asset_impl.h"