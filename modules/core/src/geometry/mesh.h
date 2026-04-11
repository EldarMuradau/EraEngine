// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core/job_system.h"
#include "core/bounding_volumes.h"

#include "asset/asset.h"
#include "asset/pbr_material_desc.h"

#include "animation/animation.h"

#include "geometry/mesh_builder.h"

namespace era_engine
{
	struct pbr_material;

	struct submesh
	{
		SubmeshInfo info;
		bounding_box aabb; // local space.
		trs transform;

		ref<pbr_material> material;
		std::string name;
	};

	struct multi_mesh
	{
		std::vector<submesh> submeshes;
		dx_mesh mesh;
		bounding_box aabb = { vec3(0.f), vec3(0.f) };

		AssetHandle handle;
		uint32 flags;

		std::atomic<AssetLoadState> loadState = AssetLoadState::LOADED;
		JobHandle loadJob;
	};

	using mesh_load_callback = std::function<void(mesh_builder& builder, std::vector<submesh>& submeshes, const bounding_box& boundingBox)>;

	ERA_CORE_API ref<multi_mesh> loadMeshFromFile(const fs::path& filename, uint32 flags = mesh_creation_flags_default, const mesh_load_callback& cb = nullptr);
	ERA_CORE_API ref<multi_mesh> loadMeshFromHandle(AssetHandle handle, uint32 flags = mesh_creation_flags_default, const mesh_load_callback& cb = nullptr);
	ERA_CORE_API ref<multi_mesh> loadMeshFromFileAsync(const fs::path& filename, uint32 flags = mesh_creation_flags_default, JobHandle parentJob = {}, const mesh_load_callback& cb = nullptr);
	ERA_CORE_API ref<multi_mesh> loadMeshFromHandleAsync(AssetHandle handle, uint32 flags = mesh_creation_flags_default, JobHandle parentJob = {}, const mesh_load_callback& cb = nullptr);

	// Same functions but with different default flags (includes skin).
	inline ref<multi_mesh> loadAnimatedMeshFromFile(const fs::path& filename, uint32 flags = mesh_creation_flags_animated, const mesh_load_callback& cb = nullptr)
	{
		return loadMeshFromFile(filename, flags, cb);
	}

	inline ref<multi_mesh> loadAnimatedMeshFromHandle(AssetHandle handle, uint32 flags = mesh_creation_flags_animated, const mesh_load_callback& cb = nullptr)
	{
		return loadMeshFromHandle(handle, flags, cb);
	}

	inline ref<multi_mesh> loadAnimatedMeshFromFileAsync(const fs::path& filename, uint32 flags = mesh_creation_flags_animated, JobHandle parentJob = {}, const mesh_load_callback& cb = nullptr)
	{
		return loadMeshFromFileAsync(filename, flags, parentJob, cb);
	}

	inline ref<multi_mesh> loadAnimatedMeshFromHandleAsync(AssetHandle handle, uint32 flags = mesh_creation_flags_animated, JobHandle parentJob = {}, const mesh_load_callback& cb = nullptr)
	{
		return loadMeshFromHandleAsync(handle, flags, parentJob, cb);
	}

}