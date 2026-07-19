#include "asset/game_asset.h"
#include "asset/asset.h"

namespace era_engine
{
	struct AssetKey
	{
		AssetHandle handle;
		uint32 flags = 0;
	};
}

namespace std
{
	template<>
	struct hash<era_engine::AssetKey>
	{
		size_t operator()(const era_engine::AssetKey& x) const
		{
			size_t seed = 0;
			hash_combine(seed, x.handle);
			hash_combine(seed, x.flags);
			return seed;
		}
	};
}

namespace era_engine
{
	static bool operator==(const AssetKey& a, const AssetKey& b)
	{
		return a.handle == b.handle && a.flags == b.flags;
	}

	static std::unordered_map<AssetKey, weakref<GameAsset>> ASSET_CACHE;
	static std::mutex ASSET_CACHE_MUTEX;

	ref<GameAsset> GameAssetCache::try_get_asset_from_cache(const fs::path& filename, uint32 flags)
	{
		if (!fs::exists(filename))
		{
			return nullptr;
		}

		AssetHandle handle = get_asset_handle_from_path(filename);

		AssetKey key = { handle, flags };

		std::lock_guard _lock{ ASSET_CACHE_MUTEX };
		ref<GameAsset> result = { ASSET_CACHE[key].lock(), {} };

		return result;
	}

	void GameAssetCache::try_add_asset_to_cache(ref<GameAsset> asset, const fs::path& filename, uint32 flags)
	{
		if (!fs::exists(filename))
		{
			return;
		}

		AssetHandle handle = get_asset_handle_from_path(filename);

		AssetKey key = { handle, flags };

		std::lock_guard _lock{ ASSET_CACHE_MUTEX };
		ref<GameAsset> result = { ASSET_CACHE[key].lock(), {} };
		if (result == nullptr)
		{
			ASSET_CACHE[key] = result;
		}
	}
}