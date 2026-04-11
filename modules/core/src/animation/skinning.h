// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core/math.h"

#include "geometry/mesh_builder.h"

namespace era_engine
{
	struct dx_vertex_buffer_group_view;
	struct dx_vertex_buffer_view;
	struct compute_pass;
}

namespace era_engine::animation
{
	struct vertex_range
	{
		uint32 firstVertex;
		uint32 numVertices;
	};

	ERA_CORE_API void initializeSkinning();
	ERA_CORE_API std::tuple<dx_vertex_buffer_group_view, mat4*> skinObject(const dx_vertex_buffer_group_view& vertexBuffer, vertex_range range, uint32 numJoints);
	ERA_CORE_API std::tuple<dx_vertex_buffer_group_view, mat4*> skinObject(const dx_vertex_buffer_group_view& vertexBuffer, uint32 numVertices, uint32 numJoints);
	ERA_CORE_API std::tuple<dx_vertex_buffer_group_view, mat4*> skinObject(const dx_vertex_buffer_group_view& vertexBuffer, SubmeshInfo submesh, uint32 numJoints);

	ERA_CORE_API dx_vertex_buffer_group_view skinCloth(const dx_vertex_buffer_view& positions, uint32 gridSizeX, uint32 gridSizeY);

	ERA_CORE_API void performSkinning(compute_pass* computePass);
}