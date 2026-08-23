#include "geometry/mesh.h"

#include <rendering/pbr.h>

#include <core/hash.h>
#include <core/string.h>
#include <core/traits.h>
#include <core/log.h>

#include <asset/file_registry.h>
#include <asset/game_asset.h>
#include <asset/model_asset.h>

#include <animation/skeleton.h>
#include <animation/animation_clip_utils.h>
#include <animation/animation_clip.h>

namespace era_engine
{
	struct MeshKey
	{
		AssetHandle handle;
		uint32 flags;
	};
}

namespace std
{
	template<>
	struct hash<era_engine::MeshKey>
	{
		size_t operator()(const era_engine::MeshKey& x) const
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
	inline std::string convert_material_path(std::string_view input_path, const std::filesystem::path& model_filepath)
	{
		std::filesystem::path model_dir = model_filepath.parent_path();

		std::filesystem::path texture_path(input_path);

		if (texture_path.is_absolute())
		{
			return texture_path.string();
		}

		std::filesystem::path full_path = (model_dir / texture_path).lexically_normal();

		std::string converted_path = full_path.string();
		replace_all(converted_path, "\\", "/");

		return converted_path;
	}

	static void mesh_import_worker(ref<MultiMesh> result, 
		const fs::path& filename, 
		uint32 flags, 
		const MeshLoadCallback& cb,
		bool async, 
		JobHandle parent_job)
	{
		using namespace animation;

		result->aabb = bounding_box::negativeInfinity();

		result->model_asset = import_3d_model_from_file(filename, flags);
		mesh_builder builder(flags | mesh_creation_flags_with_skin);

		for (PbrMaterialDesc& material_desc : result->model_asset->materials)
		{
			if (!material_desc.albedo.empty() && *material_desc.albedo.c_str() != 'F')
			{
				material_desc.albedo = convert_material_path(material_desc.albedo, filename);
			}
			if (!material_desc.normal.empty() && *material_desc.normal.c_str() != 'F')
			{
				material_desc.normal = convert_material_path(material_desc.normal, filename);
			}
			if (!material_desc.roughness.empty() && *material_desc.roughness.c_str() != 'F')
			{
				material_desc.roughness = convert_material_path(material_desc.roughness, filename);
			}
			if (!material_desc.metallic.empty() && *material_desc.metallic.c_str() != 'F')
			{
				material_desc.metallic = convert_material_path(material_desc.metallic, filename);
			}

			material_desc.emission = vec4(0.0f, 0.0f, 0.0f, 1.0f);
		}

		for (auto& mesh : result->model_asset->meshes)
		{
			if (result->model_asset->materials.empty())
			{
				result->model_asset->materials.push_back(PbrMaterialDesc{});
			}

			for (auto& sub : mesh.submeshes)
			{
				PbrMaterialDesc& material_desc = result->model_asset->materials[sub.material_index];

				ref<pbr_material> material;
				if (!async)
				{
					material = create_pbr_material(material_desc);
				}
				else
				{
					material = create_pbr_material_async(material_desc, parent_job);
				}

				bounding_box aabb;
				builder.pushMesh(sub, 1.f, &aabb);
				result->submeshes.push_back({ builder.endSubmesh(), aabb, trs::identity, material, mesh.name });

				result->aabb.grow(aabb.minCorner);
				result->aabb.grow(aabb.maxCorner);
			}
		}
		
		if (cb)
		{
			cb(builder, result->submeshes, result->aabb);
		}

		result->mesh = builder.createDXMesh();

		result->load_state.store(AssetLoadState::LOADED, std::memory_order_release);

		GameAssetsProvider provider;

		fs::path mesh_path = filename.parent_path() / filename.stem();

		JobHandle save_job = provider.save_game_asset_to_file_async<MultiMesh>(mesh_path, result.get());
		save_job.wait_for_completion();
	}

	static ref<MultiMesh> import_mesh_from_file_internal(const fs::path& filename, 
		AssetHandle handle, 
		uint32 flags, 
		const MeshLoadCallback& cb,
		bool async, 
		JobHandle parent_job)
	{
		ref<MultiMesh> result = make_ref<MultiMesh>();
		result->handle = handle;
		result->flags = flags;
		result->load_state = AssetLoadState::LOADING;

		if (!async)
		{
			mesh_import_worker(result, filename, flags, cb, false, {});
			result->load_job = {};
			return result;
		}
		else
		{
			struct MeshLoadingData
			{
				MeshLoadCallback cb;
				fs::path path;
				ref<MultiMesh> mesh;
				uint32 flags;
			};

			constexpr int a = sizeof(MeshLoadingData);

			MeshLoadingData data = { cb, filename, result, flags };

			JobHandle job = low_priority_job_queue.createJob<MeshLoadingData>([](MeshLoadingData& data, JobHandle job)
				{
					mesh_import_worker(data.mesh, data.path, data.flags, data.cb, true, job);
				}, data, parent_job);
			job.submit_now();

			result->load_job = job;

			return result;
		}
	}

	static bool operator==(const MeshKey& a, const MeshKey& b)
	{
		return a.handle == b.handle && a.flags == b.flags;
	}

	static std::unordered_map<MeshKey, weakref<MultiMesh>> mesh_cache;
	static std::mutex mutex;

	static ref<MultiMesh> import_mesh_from_file_and_handle(const fs::path& filename, 
		AssetHandle handle, 
		uint32 flags, 
		const MeshLoadCallback& cb,
		bool async = false, 
		JobHandle parent_job = {})
	{
		if (!fs::exists(filename))
		{
			return nullptr;
		}

		MeshKey key = { handle, flags };

		std::lock_guard _lock{ mutex };
		ref<MultiMesh> result = { mesh_cache[key].lock(), {} };
		if (!result)
		{
			result = import_mesh_from_file_internal(filename, handle, flags, cb, async, parent_job);
			mesh_cache[key] = result;
		}

		return result;
	}

	ref<MultiMesh> import_mesh_from_file(const fs::path& filename, uint32 flags, const MeshLoadCallback& cb)
	{
		fs::path path = filename.lexically_normal().make_preferred();

		AssetHandle handle = get_asset_handle_from_path(path);
		return import_mesh_from_file_and_handle(path, handle, flags, cb);
	}

	ref<MultiMesh> import_mesh_from_handle(AssetHandle handle, uint32 flags, const MeshLoadCallback& cb)
	{
		fs::path filename = get_path_from_asset_handle(handle);
		return import_mesh_from_file_and_handle(filename, handle, flags, cb);
	}

	ref<MultiMesh> import_mesh_from_file_async(const fs::path& filename, uint32 flags, JobHandle parent_job, const MeshLoadCallback& cb)
	{
		fs::path path = filename.lexically_normal().make_preferred();

		AssetHandle handle = get_asset_handle_from_path(path);
		return import_mesh_from_file_and_handle(path, handle, flags, cb, true, parent_job);
	}

	ref<MultiMesh> import_mesh_from_handle_async(AssetHandle handle, uint32 flags, JobHandle parent_job, const MeshLoadCallback& cb)
	{
		fs::path filename = get_path_from_asset_handle(handle);
		return import_mesh_from_file_and_handle(filename, handle, flags, cb, true, parent_job);
	}

	MultiMesh::~MultiMesh()
	{
	}

	bool MultiMesh::serialize(std::ostream& os) const
	{
		if (!model_asset.has_value())
		{
			ASSERT(model_asset.has_value());
			return false;
		}

		const BinaryDataArchive& serialized_data = BinarySerializer::serialize(model_asset);

		try
		{
			if (!IO::write_value(os, serialized_data))
			{
				return false;
			}
		}
		catch (...)
		{
			LOG_ERROR("Exception thrown while serializing asset!");
			return false;
		}

		return true;
	}

	bool MultiMesh::deserialize(std::istream& is)
	{
		try
		{
			BinaryDataArchive deserialized_data;

			if (!IO::read_value(is, deserialized_data))
			{
				return false;
			}

			if (BinarySerializer::deserialize(BinaryData(deserialized_data), model_asset) != deserialized_data.size())
			{
				return false;
			}

			aabb = bounding_box::negativeInfinity();

			mesh_builder builder(flags | mesh_creation_flags_with_skin);

			for (auto& mesh : model_asset->meshes)
			{
				if (model_asset->materials.empty())
				{
					model_asset->materials.push_back(PbrMaterialDesc{});
				}

				for (auto& sub : mesh.submeshes)
				{
					PbrMaterialDesc& material_desc = model_asset->materials[sub.material_index];
					material_desc.emission = vec4(0.0f, 0.0f, 0.0f, 1.0f);

					ref<pbr_material> material = create_pbr_material(material_desc);

					bounding_box aabb;
					builder.pushMesh(sub, 1.f, &aabb);
					submeshes.push_back({ builder.endSubmesh(), aabb, trs::identity, material, mesh.name });

					aabb.grow(aabb.minCorner);
					aabb.grow(aabb.maxCorner);
				}
			}

			mesh = builder.createDXMesh();

			if (has_flag(flags, mesh_creation_flags_compact))
			{
				model_asset.reset();
			}

			load_state.store(AssetLoadState::LOADED, std::memory_order_release);
		}
		catch (...)
		{
			LOG_ERROR("Exception thrown while serializing asset!");
			return false;
		}

		return true;
	}

	std::string MultiMesh::get_asset_type_impl()
	{
		return std::string(".emesh");
	}
}