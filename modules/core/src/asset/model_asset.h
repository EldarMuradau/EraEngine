#pragma once

#include "core_api.h"

#include "core/math.h"
#include "core/serialization/binary_serializer.h"
#include "core/bounding_volumes.h"

#include "asset/pbr_material_desc.h"

#include "animation/animation_common_data.h"

namespace era_engine
{
	struct ERA_CORE_API SubmeshAsset
	{
		int32 material_index = 0;

		std::vector<vec3> positions;
		std::vector<vec2> uvs;
		std::vector<vec3> normals;
		std::vector<vec3> tangents;
		std::vector<uint32> colors;
		std::vector<animation::SkinningWeights> skin;

		std::vector<indexed_triangle32> triangles;

		ERA_BINARY_SERIALIZE(material_index, positions, uvs, normals, tangents, colors, skin, triangles)
	};

	struct ERA_CORE_API MeshAsset
	{
		std::string name;
		std::vector<SubmeshAsset> submeshes;
		int32 skeleton_index = 0;

		ERA_BINARY_SERIALIZE(name, submeshes, skeleton_index)
	};

	struct PbrMaterialDesc;

	struct ERA_CORE_API ModelAsset
	{
		ModelAsset() = default;
		~ModelAsset() = default;

		ModelAsset(const ModelAsset&) = default;
		ModelAsset(ModelAsset&&) noexcept = default;

		ModelAsset& operator=(const ModelAsset&) = default;
		ModelAsset& operator=(ModelAsset&&) noexcept = default;

		uint32 flags = 0;
		std::vector<MeshAsset> meshes;
		std::vector<PbrMaterialDesc> materials;

		ERA_BINARY_SERIALIZE(flags, meshes, materials)
	};

	enum MeshFlags
	{
		MESH_FLAG_LOAD_UVS = (1 << 0),
		MESH_FLAG_FLIP_UVS_VERTICALLY = (1 << 1),
		MESH_FLAG_LOAD_NORNALS = (1 << 2),
		MESH_FLAG_LOAD_TANGENTS = (1 << 3),
		MESH_FLAG_GEN_NORNALS = (1 << 4), // Only if mesh has no normals.
		MESH_FLAG_GEN_TANGENTS = (1 << 5), // Only if mesh has no tangents.
		MESH_FLAG_LOAD_COLORS = (1 << 6), // Only if mesh has no tangents.
		MESH_FLAG_LOAD_SKIN = (1 << 7),

		MESH_FLAG_DEFAULT = MESH_FLAG_LOAD_UVS | MESH_FLAG_FLIP_UVS_VERTICALLY |
							MESH_FLAG_LOAD_NORNALS | MESH_FLAG_GEN_NORNALS |
							MESH_FLAG_LOAD_TANGENTS | MESH_FLAG_GEN_TANGENTS |
							MESH_FLAG_LOAD_COLORS | MESH_FLAG_LOAD_SKIN,
	};

	ERA_CORE_API ModelAsset import_3d_model_from_file(const fs::path& path, uint32 mesh_flags = MESH_FLAG_DEFAULT);

	inline bool is_mesh_extension(const fs::path& extension)
	{
		return extension == ".fbx" || extension == ".obj" || extension == ".bin";
	}

	inline bool is_mesh_extension(const std::string& extension)
	{
		return extension == ".fbx" || extension == ".obj" || extension == ".bin";
	}
}