#pragma once

#include "core/job_system.h"
#include "core/bounding_volumes.h"

#include "asset/asset.h"
#include "asset/pbr_material_desc.h"
#include "asset/game_asset.h"
#include "asset/model_asset.h"

#include "geometry/mesh_builder.h"

namespace era_engine
{
	struct pbr_material;

	struct ERA_CORE_API submesh
	{
		SubmeshInfo info;
		bounding_box aabb; // local space.
		trs transform;

		ref<pbr_material> material;
		std::string name;
	};

	class ERA_CORE_API MultiMesh : public GameAsset
	{
	public:
		~MultiMesh() override;

		bool serialize(std::ostream& os) const override;
		bool deserialize(std::istream& is) override;

		static std::string get_asset_type_impl();

		std::optional<ModelAsset> model_asset;

		std::vector<submesh> submeshes;
		dx_mesh mesh;
		bounding_box aabb = { vec3(0.f), vec3(0.f) };
	};

	using MeshLoadCallback = std::function<void(mesh_builder& builder, std::vector<submesh>& submeshes, const bounding_box& boundingBox)>;

	ERA_CORE_API ref<MultiMesh> import_mesh_from_file(const fs::path& filename, uint32 flags = mesh_creation_flags_default, const MeshLoadCallback& cb = nullptr);
	ERA_CORE_API ref<MultiMesh> import_mesh_from_handle(AssetHandle handle, uint32 flags = mesh_creation_flags_default, const MeshLoadCallback& cb = nullptr);
	ERA_CORE_API ref<MultiMesh> import_mesh_from_file_async(const fs::path& filename, uint32 flags = mesh_creation_flags_default, JobHandle parent_job = {}, const MeshLoadCallback& cb = nullptr);
	ERA_CORE_API ref<MultiMesh> import_mesh_from_handle_async(AssetHandle handle, uint32 flags = mesh_creation_flags_default, JobHandle parent_job = {}, const MeshLoadCallback& cb = nullptr);

	// Same functions but with different default flags (includes skin).
	inline ref<MultiMesh> import_animated_mesh_from_file(const fs::path& filename, uint32 flags = mesh_creation_flags_animated, const MeshLoadCallback& cb = nullptr)
	{
		return import_mesh_from_file(filename, flags, cb);
	}

	inline ref<MultiMesh> import_animated_mesh_from_handle(AssetHandle handle, uint32 flags = mesh_creation_flags_animated, const MeshLoadCallback& cb = nullptr)
	{
		return import_mesh_from_handle(handle, flags, cb);
	}

	inline ref<MultiMesh> import_animated_mesh_from_file_async(const fs::path& filename, uint32 flags = mesh_creation_flags_animated, JobHandle parent_job = {}, const MeshLoadCallback& cb = nullptr)
	{
		return import_mesh_from_file_async(filename, flags, parent_job, cb);
	}

	inline ref<MultiMesh> import_animated_mesh_from_handle_async(AssetHandle handle, uint32 flags = mesh_creation_flags_animated, JobHandle parent_job = {}, const MeshLoadCallback& cb = nullptr)
	{
		return import_mesh_from_handle_async(handle, flags, parent_job, cb);
	}

}