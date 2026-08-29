#pragma once

#include "core_api.h"

#include "core/memory.h"
#include "core/bounding_volumes.h"

#include "animation/animation_common_data.h"

#include "dx/dx_buffer.h"

namespace era_engine
{
	struct ERA_CORE_API SubmeshInfo
	{
		uint32 numIndices;
		uint32 firstIndex;
		uint32 baseVertex;
		uint32 numVertices;
	};

	// Members are always pushed in this order!
	enum mesh_creation_flags
	{
		mesh_creation_flags_none = 0,
		mesh_creation_flags_with_positions = (1 << 0),
		mesh_creation_flags_with_uvs = (1 << 1),
		mesh_creation_flags_with_normals = (1 << 2),
		mesh_creation_flags_with_tangents = (1 << 3),
		mesh_creation_flags_with_skin = (1 << 4),
		mesh_creation_flags_with_colors = (1 << 5),
		mesh_creation_flags_sm_to_m = (1 << 6),
		mesh_creation_flags_m_to_sm = (1 << 7),
		mesh_creation_flags_yzx_to_xyz = (1 << 8),
		mesh_creation_flags_generate_root_motion = (1 << 9),
		mesh_creation_flags_override_textures_with_local_folder = (1 << 10),
		mesh_creation_flags_compact = (1 << 11),

		mesh_creation_flags_default = mesh_creation_flags_with_positions | mesh_creation_flags_with_uvs | mesh_creation_flags_with_normals | mesh_creation_flags_with_tangents,
		mesh_creation_flags_animated = mesh_creation_flags_default | mesh_creation_flags_with_skin,
		mesh_creation_flags_unreal_asset = mesh_creation_flags_default | mesh_creation_flags_sm_to_m | mesh_creation_flags_yzx_to_xyz,
		mesh_creation_flags_unreal_animated_asset = mesh_creation_flags_unreal_asset | mesh_creation_flags_with_skin
	};

	enum mesh_index_type
	{
		mesh_index_uint16,
		mesh_index_uint32,
	};

	struct ERA_CORE_API quad_mesh_desc
	{
		vec3 center = 0.f;
		vec2 radius = 1.f;
		quat rotation = quat::identity;
	};

	struct ERA_CORE_API box_mesh_desc
	{
		vec3 center = 0.f;
		vec3 radius = 1.f;
		quat rotation = quat::identity;
	};

	struct ERA_CORE_API tesselated_box_mesh_desc
	{
		vec3 center = 0.f;
		vec3 radius = 1.f;
		uint32 numIntervals = 1;
		quat rotation = quat::identity;
	};

	struct ERA_CORE_API sphere_mesh_desc
	{
		vec3 center = 0.f;
		float radius = 1.f;
		uint32 slices = 15;
		uint32 rows = 15;
	};

	struct ERA_CORE_API icosphere_mesh_desc
	{
		vec3 center = 0.f;
		float radius = 1.f;
		uint32 refinement = 0;
	};

	// All meshes are standing upright by default.
	struct ERA_CORE_API capsule_mesh_desc
	{
		capsule_mesh_desc() {}

		capsule_mesh_desc(vec3 posA, vec3 posB, float _radius)
		{
			center = (posA + posB) * 0.5f;
			height = length(posA - posB);
			radius = _radius;
			rotation = rotate_from_to(vec3(0.f, 1.f, 0.f), posB - posA);
		}

		vec3 center = 0.f;
		float height = 1.f;
		float radius = 0.4f;
		quat rotation = quat::identity;
		uint32 slices = 15;
		uint32 rows = 15;
	};

	struct ERA_CORE_API cylinder_mesh_desc
	{
		vec3 center = 0.f;
		float height = 1.f;
		float radius = 0.4f;
		quat rotation = quat::identity;
		uint32 slices = 15;
	};

	struct ERA_CORE_API hollow_cylinder_mesh_desc
	{
		vec3 center = 0.f;
		float height = 1.f;
		float radius = 0.4f;
		float innerRadius = 0.3f;
		quat rotation = quat::identity;
		uint32 slices = 15;
	};

	struct ERA_CORE_API arrow_mesh_desc
	{
		vec3 base = 0.f;
		float shaftLength = 1.f;
		float shaftRadius = 0.1f;
		float headLength = 0.3f;
		float headRadius = 0.2f;
		quat rotation = quat::identity;
		uint32 slices = 15;
	};

	struct ERA_CORE_API torus_mesh_desc
	{
		vec3 center = 0.f;
		float torusRadius = 1.f;
		float tubeRadius = 0.1f;
		quat rotation = quat::identity;
		uint32 slices = 15;
		uint32 segments = 15;
	};

	struct ERA_CORE_API mace_mesh_desc
	{
		vec3 base = 0.f;
		float shaftLength = 1.f;
		float shaftRadius = 0.1f;
		float headLength = 0.3f;
		float headRadius = 0.2f;
		quat rotation = quat::identity;
		uint32 slices = 15;
	};

	NODISCARD inline constexpr uint32 getVertexOthersSize(uint32 meshFlags)
	{
		uint32 size = 0;
		if (meshFlags & mesh_creation_flags_with_uvs) { size += sizeof(vec2); }
		if (meshFlags & mesh_creation_flags_with_normals) { size += sizeof(vec3); }
		if (meshFlags & mesh_creation_flags_with_tangents) { size += sizeof(vec3); }
		if (meshFlags & mesh_creation_flags_with_skin) { size += sizeof(era_engine::animation::SkinningWeights); }
		if (meshFlags & mesh_creation_flags_with_colors) { size += sizeof(uint32); }
		return size;
	}

	struct ERA_CORE_API mesh_builder
	{
		mesh_builder(uint32 vertexFlags = mesh_creation_flags_default, mesh_index_type indexType = mesh_index_uint32);
		mesh_builder(const mesh_builder& mesh) = delete;
		mesh_builder(mesh_builder&& mesh) = default;
		~mesh_builder();

		void pushQuad(const quad_mesh_desc& desc, bool flipWindingOrder = false);
		void pushBox(const box_mesh_desc& desc, bool flipWindingOrder = false);
		void pushTesselatedBox(const tesselated_box_mesh_desc& desc, bool flipWindingOrder = false);
		void pushSphere(const sphere_mesh_desc& desc, bool flipWindingOrder = false);
		void pushIcoSphere(const icosphere_mesh_desc& desc, bool flipWindingOrder = false);
		void pushCapsule(const capsule_mesh_desc& desc, bool flipWindingOrder = false);
		void pushCylinder(const cylinder_mesh_desc& desc, bool flipWindingOrder = false);
		void pushHollowCylinder(const hollow_cylinder_mesh_desc& desc, bool flipWindingOrder = false);
		void pushArrow(const arrow_mesh_desc& desc, bool flipWindingOrder = false);
		void pushTorus(const torus_mesh_desc& desc, bool flipWindingOrder = false);
		void pushMace(const mace_mesh_desc& desc, bool flipWindingOrder = false);

		void pushMesh(const struct SubmeshAsset& mesh, float scale, bounding_box* aabb = 0);

		SubmeshInfo endSubmesh();

		dx_mesh createDXMesh();

		NODISCARD vec3* getPositions() { return (vec3*)positionArena.base(); }
		NODISCARD void* getOthers() { return othersArena.base(); }
		NODISCARD void* getTriangles() { return indexArena.base(); }

		NODISCARD uint32 getNumVertices() const { return totalNumVertices; }
		NODISCARD uint32 getNumTriangles() const { return totalNumTriangles; }
		NODISCARD uint32 getSkinOffset() const { return skinOffset; }
		NODISCARD uint32 getOthersSize() const { return othersSize; }

	private:
		std::tuple<vec3*, uint8*, uint8*, uint32> beginPrimitive(uint32 numVertices, uint32 numTriangles);

		Allocator positionArena;
		Allocator othersArena;
		Allocator indexArena;

		uint32 vertexFlags;
		mesh_index_type indexType;
		uint32 othersSize;
		uint32 indexSize;
		uint32 skinOffset;

		uint32 totalNumVertices = 0;
		uint32 totalNumTriangles = 0;

		uint32 numVerticesInCurrentSubmesh = 0;
		uint32 numTrianglesInCurrentSubmesh = 0;

		uint32 numSubmeshes = 0;
	};

	struct vertex_uv_normal_tangent
	{
		vec2 uv;
		vec3 normal;
		vec3 tangent;
	};

	static D3D12_INPUT_ELEMENT_DESC inputLayout_position[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
	};

	static D3D12_INPUT_ELEMENT_DESC inputLayout_position_uv[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "TEXCOORDS", 0, DXGI_FORMAT_R32G32_FLOAT, 1, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
	};

	static D3D12_INPUT_ELEMENT_DESC inputLayout_position_normal[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 1, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
	};

	static D3D12_INPUT_ELEMENT_DESC inputLayout_position_uv_normal[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "TEXCOORDS", 0, DXGI_FORMAT_R32G32_FLOAT, 1, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 1, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
	};

	static D3D12_INPUT_ELEMENT_DESC inputLayout_position_uv_normal_tangent[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "TEXCOORDS", 0, DXGI_FORMAT_R32G32_FLOAT, 1, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 1, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "TANGENT", 0, DXGI_FORMAT_R32G32B32_FLOAT, 1, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
	};

	static D3D12_INPUT_ELEMENT_DESC inputLayout_position_uv_normal_tangent_colors[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "TEXCOORDS", 0, DXGI_FORMAT_R32G32_FLOAT, 1, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 1, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "TANGENT", 0, DXGI_FORMAT_R32G32B32_FLOAT, 1, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 1, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
	};
}