// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core_api.h"
#include "core/camera.h"

#include "rendering/render_pass.h"

#include "dx/dx_command_list.h"

#include "visualization_rs.hlsli"

namespace era_engine
{
	struct ERA_CORE_API position_color
	{
		vec3 position;
		vec3 color;
	};

	struct debug_render_data
	{
		mat4 transform;
		dx_vertex_buffer_group_view vertexBuffer;
		dx_index_buffer_view indexBuffer;
		SubmeshInfo submesh;

		vec4 color;
	};

	struct debug_simple_pipeline
	{
		// Layout: position, normal.
		static void initialize();

		PIPELINE_SETUP_DECL;
		PIPELINE_RENDER_DECL(debug_render_data);
	};

	struct ERA_CORE_API debug_unlit_pipeline
	{
		static void initialize();

		struct position;
		struct position_color;

		PIPELINE_RENDER_DECL(debug_render_data);
	};

	struct ERA_CORE_API debug_unlit_pipeline::position : debug_unlit_pipeline { PIPELINE_SETUP_DECL; };
	struct ERA_CORE_API debug_unlit_pipeline::position_color : debug_unlit_pipeline { PIPELINE_SETUP_DECL; };

	struct ERA_CORE_API debug_unlit_line_pipeline
	{
		static void initialize();

		struct position;
		struct position_color;

		PIPELINE_RENDER_DECL(debug_render_data);
	};

	struct ERA_CORE_API debug_unlit_line_pipeline::position : debug_unlit_line_pipeline { PIPELINE_SETUP_DECL; };
	struct ERA_CORE_API debug_unlit_line_pipeline::position_color : debug_unlit_line_pipeline { PIPELINE_SETUP_DECL; };

	template <typename pipeline_t>
	static void renderDebug(const mat4& transform, const struct dx_dynamic_vertex_buffer& vb, const struct dx_dynamic_index_buffer& ib, vec4 color,
		ldr_render_pass* renderPass, bool overlay = false)
	{
		SubmeshInfo sm;
		sm.baseVertex = 0;
		sm.numVertices = vb.view.SizeInBytes / vb.view.StrideInBytes;
		sm.firstIndex = 0;
		sm.numIndices = ib.view.SizeInBytes / getFormatSize(ib.view.Format);

		debug_render_data data = {
			transform, dx_vertex_buffer_group_view(vb), ib, sm, color
		};

		if (overlay)
			renderPass->renderOverlay<pipeline_t>(data);
		else
			renderPass->renderObject<pipeline_t>(data);
	}

	ERA_CORE_API void renderPoint(vec3 pos, vec4 color, ldr_render_pass* renderPass, bool overlay = false);
	ERA_CORE_API void renderTriangle(vec3 a, vec3 b, vec3 c, vec4 color, ldr_render_pass* renderPass, bool overlay = false);
	ERA_CORE_API void renderDisk(vec3 position, vec3 upAxis, float radius, vec4 color, ldr_render_pass* renderPass, bool overlay = false);
	ERA_CORE_API void renderRing(vec3 position, vec3 upAxis, float outerRadius, float innerRadius, vec4 color, ldr_render_pass* renderPass, bool overlay = false);
	ERA_CORE_API void renderAngleRing(vec3 position, vec3 upAxis, float outerRadius, float innerRadius,
		vec3 zeroDegAxis, float minAngle, float maxAngle, vec4 color, ldr_render_pass* renderPass, bool overlay = false);

	ERA_CORE_API void renderLine(vec3 positionA, vec3 positionB, vec4 color, ldr_render_pass* renderPass, bool overlay = false);
	ERA_CORE_API void renderWireTriangle(vec3 a, vec3 b, vec3 c, vec4 color, ldr_render_pass* renderPass, bool overlay = false);
	ERA_CORE_API void renderWireSphere(vec3 position, float radius, vec4 color, ldr_render_pass* renderPass, bool overlay = false);
	ERA_CORE_API void renderWireCapsule(vec3 positionA, vec3 positionB, float radius, vec4 color, ldr_render_pass* renderPass, bool overlay = false);
	ERA_CORE_API void renderWireCylinder(vec3 positionA, vec3 positionB, float radius, vec4 color, ldr_render_pass* renderPass, bool overlay = false);
	ERA_CORE_API void renderWireCone(vec3 position, vec3 direction, float distance, float angle, vec4 color, ldr_render_pass* renderPass, bool overlay = false);
	ERA_CORE_API void renderWireBox(vec3 position, vec3 radius, quat rotation, vec4 color, ldr_render_pass* renderPass, bool overlay = false);
	ERA_CORE_API void renderCameraFrustum(const render_camera& frustum, vec4 color, ldr_render_pass* renderPass, float alternativeFarPlane = -1.f, bool overlay = false);
}