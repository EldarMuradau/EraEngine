// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "asset/bin.h"

#include "asset/model_asset.h"
#include "asset/asset.h"
#include "asset/io.h"

#include "core/cpu_profiling.h"

#include "rendering/pbr_material.h"

#define PROFILE(name) ZoneScopedN(name)

namespace era_engine
{
	struct bin_header
	{
		BinaryHeader header;
		uint32 flags;
		uint32 numMeshes;
		uint32 numMaterials;
		uint32 numSkeletons;
		uint32 numAnimations;
	};

	struct bin_mesh_header
	{
		uint32 numSubmeshes;
		int32 skeletonIndex;
		uint32 nameLength;
	};

	enum bin_submesh_flag
	{
		bin_submesh_flag_positions = (1 << 0),
		bin_submesh_flag_uvs = (1 << 1),
		bin_submesh_flag_normals = (1 << 2),
		bin_submesh_flag_tangents = (1 << 3),
		bin_submesh_flag_colors = (1 << 4),
		bin_submesh_flag_skin = (1 << 5),
	};

	struct bin_submesh_header
	{
		uint32 numVertices;
		uint32 numTriangles;
		int32 materialIndex;
		uint32 flags;
	};

	struct bin_material_header
	{
		uint32 albedoPathLength;
		uint32 normalPathLength;
		uint32 roughnessPathLength;
		uint32 metallicPathLength;
	};

	struct bin_skeleton_header
	{
		uint32 numJoints;
	};

	struct bin_animation_header
	{
		float duration;
		uint32 numJoints;
		uint32 numPositionKeyframes;
		uint32 numRotationKeyframes;
		uint32 numScaleKeyframes;

		uint32 nameLength;
	};

	template <typename T>
	static void writeArray(const std::vector<T>& in, FILE* file)
	{
		fwrite(in.data(), sizeof(T), in.size(), file);
	}

	static void writeMesh(const MeshAsset& mesh, FILE* file)
	{
		bin_mesh_header header;
		header.skeletonIndex = mesh.skeleton_index;
		header.numSubmeshes = (uint32)mesh.submeshes.size();
		header.nameLength = (uint32)mesh.name.length();

		fwrite(&header, sizeof(header), 1, file);
		fwrite(mesh.name.c_str(), sizeof(char), header.nameLength, file);

		for (uint32 i = 0; i < header.numSubmeshes; ++i)
		{
			const SubmeshAsset& in = mesh.submeshes[i];

			bin_submesh_header subHeader;
			subHeader.materialIndex = in.material_index;
			subHeader.numVertices = (uint32)in.positions.size();
			subHeader.numTriangles = (uint32)in.triangles.size();

			subHeader.flags = 0;
			if (!in.positions.empty()) { subHeader.flags |= bin_submesh_flag_positions; }
			if (!in.uvs.empty()) { subHeader.flags |= bin_submesh_flag_uvs; }
			if (!in.normals.empty()) { subHeader.flags |= bin_submesh_flag_normals; }
			if (!in.tangents.empty()) { subHeader.flags |= bin_submesh_flag_tangents; }
			if (!in.colors.empty()) { subHeader.flags |= bin_submesh_flag_colors; }
			if (!in.skin.empty()) { subHeader.flags |= bin_submesh_flag_skin; }

			fwrite(&subHeader, sizeof(bin_submesh_header), 1, file);

			if (!in.positions.empty()) { writeArray(in.positions, file); }
			if (!in.uvs.empty()) { writeArray(in.uvs, file); }
			if (!in.normals.empty()) { writeArray(in.normals, file); }
			if (!in.tangents.empty()) { writeArray(in.tangents, file); }
			if (!in.colors.empty()) { writeArray(in.colors, file); }
			if (!in.skin.empty()) { writeArray(in.skin, file); }
			writeArray(in.triangles, file);
		}
	}

	static void writeMaterial(const PbrMaterialDesc& material, FILE* file)
	{
		const std::string& albedo = material.albedo;
		const std::string& normal = material.normal;
		const std::string& roughness = material.roughness;
		const std::string& metallic = material.metallic;

		bin_material_header header;
		header.albedoPathLength = (uint32)albedo.length();
		header.normalPathLength = (uint32)normal.length();
		header.roughnessPathLength = (uint32)roughness.length();
		header.metallicPathLength = (uint32)metallic.length();

		fwrite(&header, sizeof(header), 1, file);
		fwrite(albedo.c_str(), sizeof(char), header.albedoPathLength, file);
		fwrite(normal.c_str(), sizeof(char), header.normalPathLength, file);
		fwrite(roughness.c_str(), sizeof(char), header.roughnessPathLength, file);
		fwrite(metallic.c_str(), sizeof(char), header.metallicPathLength, file);

		fwrite(&material.albedo_flags, sizeof(uint32), 1, file);
		fwrite(&material.normal_flags, sizeof(uint32), 1, file);
		fwrite(&material.roughness_flags, sizeof(uint32), 1, file);
		fwrite(&material.metallic_flags, sizeof(uint32), 1, file);

		fwrite(&material.emission, sizeof(vec4), 1, file);
		fwrite(&material.albedo_tint, sizeof(vec4), 1, file);
		fwrite(&material.roughness_override, sizeof(float), 1, file);
		fwrite(&material.metallic_override, sizeof(float), 1, file);
		fwrite(&material.shader, sizeof(PbrMaterialShader), 1, file);
		fwrite(&material.uv_scale, sizeof(float), 1, file);
		fwrite(&material.translucency, sizeof(float), 1, file);
	}

	void write_bin(const ModelAsset& asset, const fs::path& path)
	{
		FILE* file = fopen(path.string().c_str(), "wb");

		bin_header header;
		header.flags = asset.flags;
		header.numMeshes = (uint32)asset.meshes.size();
		header.numMaterials = (uint32)asset.materials.size();

		fwrite(&header, sizeof(header), 1, file);

		for (uint32 i = 0; i < header.numMeshes; ++i)
		{
			writeMesh(asset.meshes[i], file);
		}
		for (uint32 i = 0; i < header.numMaterials; ++i)
		{
			writeMaterial(asset.materials[i], file);
		}

		fclose(file);
	}

	template <typename T>
	static void readArray(EntireFile& file, std::vector<T>& out, uint32 count)
	{
		out.resize(count);
		T* ptr = file.consume<T>(count);
		memcpy(out.data(), ptr, sizeof(T) * count);
	}

	static MeshAsset readMesh(EntireFile& file)
	{
		bin_mesh_header* header = file.consume<bin_mesh_header>();
		char* name = file.consume<char>(header->nameLength);

		MeshAsset result;
		result.name = std::string(name, header->nameLength);
		result.submeshes.resize(header->numSubmeshes);
		result.skeleton_index = header->skeletonIndex;

		for (uint32 i = 0; i < header->numSubmeshes; ++i)
		{
			bin_submesh_header* subHeader = file.consume<bin_submesh_header>();

			SubmeshAsset& sub = result.submeshes[i];
			sub.material_index = subHeader->materialIndex;
			if (subHeader->flags & bin_submesh_flag_positions) { readArray(file, sub.positions, subHeader->numVertices); }
			if (subHeader->flags & bin_submesh_flag_uvs) { readArray(file, sub.uvs, subHeader->numVertices); }
			if (subHeader->flags & bin_submesh_flag_normals) { readArray(file, sub.normals, subHeader->numVertices); }
			if (subHeader->flags & bin_submesh_flag_tangents) { readArray(file, sub.tangents, subHeader->numVertices); }
			if (subHeader->flags & bin_submesh_flag_colors) { readArray(file, sub.colors, subHeader->numVertices); }
			if (subHeader->flags & bin_submesh_flag_skin) { readArray(file, sub.skin, subHeader->numVertices); }
			readArray(file, sub.triangles, subHeader->numTriangles);
		}

		return result;
	}

	static PbrMaterialDesc readMaterial(EntireFile& file)
	{
		bin_material_header* header = file.consume<bin_material_header>();

		char* albedo = file.consume<char>(header->albedoPathLength);
		char* normal = file.consume<char>(header->normalPathLength);
		char* roughness = file.consume<char>(header->roughnessPathLength);
		char* metallic = file.consume<char>(header->metallicPathLength);

		PbrMaterialDesc result;
		result.albedo = std::string(albedo, header->albedoPathLength);
		result.normal = std::string(normal, header->normalPathLength);
		result.roughness = std::string(roughness, header->roughnessPathLength);
		result.metallic = std::string(metallic, header->metallicPathLength);

		result.albedo_flags = *file.consume<uint32>();
		result.normal_flags = *file.consume<uint32>();
		result.roughness_flags = *file.consume<uint32>();
		result.metallic_flags = *file.consume<uint32>();

		result.emission = *file.consume<vec4>();
		result.albedo_tint = *file.consume<vec4>();
		result.roughness_override = *file.consume<float>();
		result.metallic_override = *file.consume<float>();
		result.shader = *file.consume<PbrMaterialShader>();
		result.uv_scale = *file.consume<float>();
		result.translucency = *file.consume<float>();

		return result;
	}

	ModelAsset load_bin(const fs::path& path)
	{
		PROFILE("Loading BIN");

		EntireFile file = load_file(path);

		bin_header* header = file.consume<bin_header>();
		if (header->header.header != BIN_HEADER)
		{
			free_file(file);
			return {};
		}

		ModelAsset result;
		result.meshes.resize(header->numMeshes);
		result.materials.resize(header->numMaterials);

		for (uint32 i = 0; i < header->numMeshes; ++i)
		{
			result.meshes[i] = readMesh(file);
		}
		for (uint32 i = 0; i < header->numMaterials; ++i)
		{
			result.materials[i] = readMaterial(file);
		}

		free_file(file);

		return result;
	}
}