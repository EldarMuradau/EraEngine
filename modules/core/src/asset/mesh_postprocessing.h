// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core_api.h"

#include "core/math.h"
#include "core/hash.h"

#include "geometry/mesh.h"

#include "asset/model_asset.h"

namespace era_engine
{
	struct ERA_CORE_API full_vertex
	{
		vec3 position;
		vec2 uv;
		vec3 normal;
		vec3 tangent;
		uint32 color;
		animation::SkinningWeights skin;
	};

	static bool operator==(const full_vertex& a, const full_vertex& b)
	{
		return memcmp(&a, &b, sizeof(full_vertex)) == 0;
	}
}

namespace std
{
	template<>
	struct hash<era_engine::full_vertex>
	{
		size_t operator()(const era_engine::full_vertex& x) const
		{
			size_t seed = 0;

			hash_combine(seed, x.position);
			hash_combine(seed, x.uv);
			hash_combine(seed, x.normal);
			hash_combine(seed, x.tangent);
			hash_combine(seed, x.color);
			hash_combine(seed, *(uint32*)x.skin.skin_indices.data());
			hash_combine(seed, *(uint32*)x.skin.skin_weights.data());

			return seed;
		}
	};
}

namespace era_engine
{
	struct ERA_CORE_API per_material
	{
		void addTriangles(const std::vector<vec3>& positions, const std::vector<vec2>& uvs, const std::vector<vec3>& normals,
			const std::vector<vec3>& tangents, const std::vector<uint32>& colors, const std::vector<animation::SkinningWeights>& skins,
			int32 firstIndex, int32 faceSize, std::vector<SubmeshAsset>& outSubmeshes);

		void flush(std::vector<SubmeshAsset>& outSubmeshes);

		std::unordered_map<full_vertex, uint16> vertexToIndex;
		SubmeshAsset sub;

	private:
		struct add_vertex_result
		{
			uint16 index;
			bool success;
		};

		add_vertex_result addVertex(const std::vector<vec3>& positions, const std::vector<vec2>& uvs, const std::vector<vec3>& normals,
			const std::vector<vec3>& tangents, const std::vector<uint32>& colors, const std::vector<animation::SkinningWeights>& skins,
			int32 index);
	};

	ERA_CORE_API void generateNormalsAndTangents(std::vector<SubmeshAsset>& submeshes, uint32 flags);
	ERA_CORE_API void generateNormalsAndTangents(ref<SubmeshAsset> submesh, uint32 flags);
}