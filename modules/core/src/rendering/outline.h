// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core/math.h"

#include "geometry/mesh.h"

namespace era_engine
{
	void initializeOutlinePipelines();
	void renderOutline(struct ldr_render_pass* renderPass, const mat4& transform, dx_vertex_buffer_view vertexBuffer, dx_index_buffer_view indexBuffer, SubmeshInfo submesh);
	void renderOutline(struct ldr_render_pass* renderPass, const mat4& transform, dx_vertex_buffer_group_view vertexBuffer, dx_index_buffer_view indexBuffer, SubmeshInfo submesh);
}