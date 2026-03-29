// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "terrain/grass.h"

#include "core/math.h"

#include "rendering/render_command.h"
#include "rendering/material.h"
#include "rendering/render_utils.h"
#include "rendering/render_resources.h"

#include "dx/dx_command_list.h"
#include "dx/dx_profiling.h"
#include "dx/dx_barrier_batcher.h"

#include <rttr/registration>

#include "grass_rs.hlsli"
#include "depth_only_rs.hlsli"

namespace era_engine
{
	static const uint32 numSegmentsLOD0 = 4;

	static dx_pipeline grassPipeline;
	static dx_pipeline grassDepthOnlyPipeline;

	static dx_pipeline grassNoDepthPrepassPipeline;

	static dx_pipeline grassGenerationPipeline;
	static dx_pipeline grassCreateDrawCallsPipeline;

	static dx_command_signature grassCommandSignature;

	void initializeGrassPipelines()
	{
		{
			auto& desc = CREATE_GRAPHICS_PIPELINE
				.depthSettings(true, false, D3D12_COMPARISON_FUNC_EQUAL)
				.cullingOff()
				.renderTargets(opaqueLightPassFormats, OPQAUE_LIGHT_PASS_NO_VELOCITIES_NO_OBJECT_ID, depthStencilFormat);

			grassPipeline = createReloadablePipeline(desc, { "grass_vs", "grass_ps" });
		}
		{
			auto& desc = CREATE_GRAPHICS_PIPELINE
				.cullingOff()
				.renderTargets(depthOnlyFormat, arraysize(depthOnlyFormat), depthStencilFormat);

			grassDepthOnlyPipeline = createReloadablePipeline(desc, { "grass_depth_only_vs", "depth_only_ps" }, rs_in_vertex_shader);
		}
		{
			auto& desc = CREATE_GRAPHICS_PIPELINE
				.cullingOff()
				.renderTargets(opaqueLightPassFormats, OPQAUE_LIGHT_PASS_FULL, depthStencilFormat);

			grassNoDepthPrepassPipeline = createReloadablePipeline(desc, { "grass_no_depth_prepass_vs", "grass_no_depth_prepass_ps" });
		}

		grassGenerationPipeline = createReloadablePipeline("grass_generation_cs");
		grassCreateDrawCallsPipeline = createReloadablePipeline("grass_create_draw_calls_cs");


		D3D12_INDIRECT_ARGUMENT_DESC argumentDesc;
		argumentDesc.Type = D3D12_INDIRECT_ARGUMENT_TYPE_DRAW;
		grassCommandSignature = createCommandSignature({}, &argumentDesc, 1, sizeof(grass_draw));
	}

	static grass_cb createGrassCB(const grass_settings& settings, vec2 windDirection, uint32 numVertices)
	{
		grass_cb cb;
		cb.numVertices = numVertices;
		cb.halfWidth = settings.bladeWidth * 0.5f;
		cb.windDirection = windDirection;
		return cb;
	}

	static grass_cb createGrassCB_LOD0(const grass_settings& settings, vec2 windDirection)
	{
		const uint32 numVerticesLOD0 = numSegmentsLOD0 * 2 + 1;
		return createGrassCB(settings, windDirection, numVerticesLOD0);
	}

	static grass_cb createGrassCB_LOD1(const grass_settings& settings, vec2 windDirection)
	{
		const uint32 numVerticesLOD0 = numSegmentsLOD0 * 2 + 1;
		const uint32 numVerticesLOD1 = numVerticesLOD0 / 2 + 1;
		return createGrassCB(settings, windDirection, numVerticesLOD1);
	}

	struct grass_render_data
	{
		grass_settings settings;
		ref<dx_buffer> drawBuffer;
		ref<dx_buffer> bladeBufferLOD0;
		ref<dx_buffer> bladeBufferLOD1;

		vec2 windDirection;

		uint32 objectID;
	};

	static void setup(dx_command_list* cl, const common_render_data& common, const dx_pipeline& pipeline)
	{
		cl->setPipelineState(*pipeline.pipeline);
		cl->setGraphicsRootSignature(*pipeline.rootSignature);
		cl->setPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
		cl->setGraphicsDynamicConstantBuffer(GRASS_RS_CAMERA, common.cameraCBV);
		cl->setGraphicsDynamicConstantBuffer(GRASS_RS_LIGHTING, common.lightingCBV);


		dx_cpu_descriptor_handle nullTexture = render_resources::nullTextureSRV;

		cl->setDescriptorHeapSRV(GRASS_RS_FRAME_CONSTANTS, 0, common.irradiance);
		cl->setDescriptorHeapSRV(GRASS_RS_FRAME_CONSTANTS, 1, common.prefilteredRadiance);
		cl->setDescriptorHeapSRV(GRASS_RS_FRAME_CONSTANTS, 2, render_resources::brdfTex);
		cl->setDescriptorHeapSRV(GRASS_RS_FRAME_CONSTANTS, 3, common.shadowMap);
		cl->setDescriptorHeapSRV(GRASS_RS_FRAME_CONSTANTS, 4, common.aoTexture ? common.aoTexture : render_resources::whiteTexture);
		cl->setDescriptorHeapSRV(GRASS_RS_FRAME_CONSTANTS, 5, common.sssTexture ? common.sssTexture : render_resources::whiteTexture);
		cl->setDescriptorHeapSRV(GRASS_RS_FRAME_CONSTANTS, 6, common.ssrTexture ? common.ssrTexture->defaultSRV : nullTexture);
	}

	static void render(dx_command_list* cl, const grass_render_data& data, uint32 cbRootParameterIndex, uint32 bladeRootParameterIndex)
	{
		{
			grass_cb cb = createGrassCB_LOD0(data.settings, data.windDirection);

			cl->setRootGraphicsSRV(bladeRootParameterIndex, data.bladeBufferLOD0);
			cl->setGraphics32BitConstants(cbRootParameterIndex, cb);

			cl->drawIndirect(grassCommandSignature, 1, data.drawBuffer, 0 * sizeof(grass_draw));
		}

		{
			grass_cb cb = createGrassCB_LOD1(data.settings, data.windDirection);

			cl->setRootGraphicsSRV(bladeRootParameterIndex, data.bladeBufferLOD1);
			cl->setGraphics32BitConstants(cbRootParameterIndex, cb);

			cl->drawIndirect(grassCommandSignature, 1, data.drawBuffer, 1 * sizeof(grass_draw));
		}
	}

	struct grass_pipeline
	{
		PIPELINE_SETUP_DECL
		{
			era_engine::setup(cl, common, grassPipeline);
		}

		PIPELINE_RENDER_DECL(grass_render_data)
		{
			PROFILE_ALL(cl, "Grass");
			era_engine::render(cl, data, GRASS_RS_CB, GRASS_RS_BLADES);
		}
	};

	struct grass_depth_prepass_pipeline
	{
		PIPELINE_SETUP_DECL
		{
			cl->setPipelineState(*grassDepthOnlyPipeline.pipeline);
			cl->setGraphicsRootSignature(*grassDepthOnlyPipeline.rootSignature);
			cl->setPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);

			cl->setGraphicsDynamicConstantBuffer(GRASS_DEPTH_ONLY_RS_CAMERA, common.cameraCBV);
		}

		DEPTH_ONLY_RENDER_DECL(grass_render_data)
		{
			PROFILE_ALL(cl, "Grass depth prepass");

			cl->setGraphics32BitConstants(GRASS_DEPTH_ONLY_RS_OBJECT_ID, data.objectID);
			era_engine::render(cl, data, GRASS_DEPTH_ONLY_RS_CB, GRASS_DEPTH_ONLY_RS_BLADES);
		}
	};

	struct grass_no_depth_prepass_pipeline
	{
		PIPELINE_SETUP_DECL
		{
			era_engine::setup(cl, common, grassNoDepthPrepassPipeline);
		}

		PIPELINE_RENDER_DECL(grass_render_data)
		{
			PROFILE_ALL(cl, "Grass");

			cl->setGraphics32BitConstants(GRASS_RS_OBJECT_ID, data.objectID);
			era_engine::render(cl, data, GRASS_RS_CB, GRASS_RS_BLADES);
		}
	};

	struct grass_update_data
	{
		struct grass_update_terrain_chunk
		{
			ref<dx_texture> heightmap;
			ref<dx_texture> normalmap;
		};

		std::vector<grass_update_terrain_chunk> chunks;

		grass_settings settings;

		camera_frustum_planes cameraFrustum;
		vec3 cameraPosition;

		vec3 minCorner;
		uint32 chunksPerDim;
		float chunkSize;
		float amplitudeScale;

		float time;
		float prevTime;

		ref<dx_buffer> drawBuffer;
		ref<dx_buffer> countBuffer;
		ref<dx_buffer> bladeBufferLOD0;
		ref<dx_buffer> bladeBufferLOD1;
	};

	struct grass_update_pipeline
	{
		PIPELINE_COMPUTE_DECL(grass_update_data)
		{
			PROFILE_ALL(cl, "Grass generation");

			{
				PROFILE_ALL(cl, "Generate blades");

				cl->setPipelineState(*grassGenerationPipeline.pipeline);
				cl->setComputeRootSignature(*grassGenerationPipeline.rootSignature);

				vec3 minCorner = data.minCorner;
				vec3 chunkSize(data.chunkSize, data.amplitudeScale, data.chunkSize);

				cl->setDescriptorHeap(dxContext.srvUavAllocator.type, dxContext.srvUavAllocatorShaderVisible.getHeap(0).Get());
				cl->clearUAV(data.countBuffer, 0u);
				cl->resetToDynamicDescriptorHeap();

				uint32 numGrassBladesPerDim = data.settings.numGrassBladesPerChunkDim & (~1); // Make sure this is an even number.
				numGrassBladesPerDim = min(numGrassBladesPerDim, 1024u);
				const float lodChangeEndDistance = data.settings.lodChangeStartDistance + data.settings.lodChangeTransitionDistance;
				const float cullEndDistance = data.settings.cullStartDistance + data.settings.cullTransitionDistance;

				grass_generation_common_cb common;
				memcpy(common.frustumPlanes, data.cameraFrustum.planes, sizeof(vec4) * 6);
				common.amplitudeScale = data.amplitudeScale;
				common.chunkSize = data.chunkSize;
				common.cameraPosition = data.cameraPosition;
				common.lodChangeStartDistance = data.settings.lodChangeStartDistance;
				common.lodChangeEndDistance = lodChangeEndDistance;
				common.cullStartDistance = data.settings.cullStartDistance;
				common.cullEndDistance = cullEndDistance;
				common.uvScale = 1.f / numGrassBladesPerDim;
				common.baseHeight = data.settings.bladeHeight;
				common.time = data.time;
				common.prevFrameTime = data.prevTime;

				auto commonCBV = dxContext.uploadDynamicConstantBuffer(common);
				cl->setComputeDynamicConstantBuffer(GRASS_GENERATION_RS_COMMON, commonCBV);


				for (uint32 z = 0; z < data.chunksPerDim; ++z)
				{
					for (uint32 x = 0; x < data.chunksPerDim; ++x)
					{
						auto& chunk = data.chunks[z * data.chunksPerDim + x];
						vec3 chunkMinCorner = minCorner + vec3(x * data.chunkSize, 0.f, z * data.chunkSize);
						vec3 chunkMaxCorner = chunkMinCorner + chunkSize;

						bounding_box aabb = { chunkMinCorner, chunkMaxCorner };
						if (!data.cameraFrustum.cullWorldSpaceAABB(aabb))
						{
							uint32 chunkNumGrassBladesPerDim = numGrassBladesPerDim;
							uint32 lodIndex = 0;

							float sqDistance = point_in_box(data.cameraPosition, aabb.minCorner, aabb.maxCorner)
								? 0.f
								: squared_length(data.cameraPosition - closestPoint_PointAABB(data.cameraPosition, aabb));
							if (sqDistance > lodChangeEndDistance * lodChangeEndDistance)
							{
								chunkNumGrassBladesPerDim /= 2;
								lodIndex = 1;
							}
							if (sqDistance > cullEndDistance * cullEndDistance)
							{
								continue;
							}

							cl->setDescriptorHeapSRV(GRASS_GENERATION_RS_RESOURCES, 0, chunk.heightmap);
							cl->setDescriptorHeapSRV(GRASS_GENERATION_RS_RESOURCES, 1, chunk.normalmap);
							cl->setDescriptorHeapUAV(GRASS_GENERATION_RS_RESOURCES, 2, data.bladeBufferLOD0);
							cl->setDescriptorHeapUAV(GRASS_GENERATION_RS_RESOURCES, 3, data.bladeBufferLOD1);
							cl->setDescriptorHeapUAV(GRASS_GENERATION_RS_RESOURCES, 4, data.countBuffer);

							cl->setCompute32BitConstants(GRASS_GENERATION_RS_CB, grass_generation_cb{ chunkMinCorner, lodIndex });

							cl->dispatch(bucketize(chunkNumGrassBladesPerDim, GRASS_GENERATION_BLOCK_SIZE), bucketize(chunkNumGrassBladesPerDim, GRASS_GENERATION_BLOCK_SIZE), 1);

							//barrier_batcher(cl)
							//	.uav(data.bladeBufferLOD0)
							//	.uav(data.bladeBufferLOD1)
							//	.uav(data.countBuffer);
						}
					}
				}
			}

			cl->transitionBarrier(data.countBuffer, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

			{
				PROFILE_ALL(cl, "Create draw calls");

				cl->setPipelineState(*grassCreateDrawCallsPipeline.pipeline);
				cl->setComputeRootSignature(*grassCreateDrawCallsPipeline.rootSignature);

				cl->setCompute32BitConstants(GRASS_CREATE_DRAW_CALLS_RS_CB, grass_create_draw_calls_cb{ data.bladeBufferLOD0->elementCount });
				cl->setRootComputeUAV(GRASS_CREATE_DRAW_CALLS_RS_OUTPUT, data.drawBuffer);
				cl->setRootComputeSRV(GRASS_CREATE_DRAW_CALLS_RS_MESH_COUNTS, data.countBuffer);

				cl->dispatch(bucketize(data.drawBuffer->elementCount, GRASS_CREATE_DRAW_CALLS_BLOCK_SIZE));
			}
		}
	};

	RTTR_REGISTRATION
	{
		using namespace rttr;
		rttr::registration::class_<GrassComponent>("GrassComponent")
			.constructor<>()
			.constructor<ref<Entity::EcsData>, const grass_settings&>()
			.property("settings", &GrassComponent::settings);
	}

	GrassComponent::GrassComponent(ref<Entity::EcsData> _data, const grass_settings& _settings)
		: Component(_data), settings(_settings)
	{
		uint32 num_vertices_LOD0 = numSegmentsLOD0 * 2 + 1;
		uint32 num_vertices_LOD1 = num_vertices_LOD0 / 2 + 1;

		grass_draw draw[2] = {};
		draw[0].draw.VertexCountPerInstance = num_vertices_LOD0;
		draw[1].draw.VertexCountPerInstance = num_vertices_LOD1;

		draw_buffer = createBuffer(sizeof(grass_draw), 2, &draw, true);
		count_buffer = createBuffer(sizeof(uint32), 2, 0, true, true);
		blade_buffer_LOD0 = createBuffer(sizeof(grass_blade), 1000000, 0, true);
		blade_buffer_LOD1 = createBuffer(sizeof(grass_blade), 1000000, 0, true);
	}

	GrassComponent::~GrassComponent()
	{
	}

	void GrassComponent::generate(compute_pass* compute_pass, const render_camera& camera, const TerrainComponent& terrain, vec3 position_offset, float dt)
	{
		prev_time = time;
		time += dt;

		grass_update_data data;
		data.chunks.reserve(terrain.chunks.size());
		for (const auto& chunk : terrain.chunks)
		{
			data.chunks.push_back({ chunk.heightmap, chunk.normalmap });
		}

		data.settings = settings;
		data.cameraFrustum = camera.getWorldSpaceFrustumPlanes();
		data.cameraPosition = camera.position;

		data.minCorner = terrain.get_min_corner(position_offset);
		data.chunksPerDim = terrain.chunksPerDim;
		data.chunkSize = terrain.chunkSize;
		data.amplitudeScale = terrain.amplitudeScale;

		data.time = time;
		data.prevTime = prev_time;

		data.drawBuffer = draw_buffer;
		data.countBuffer = count_buffer;
		data.bladeBufferLOD0 = blade_buffer_LOD0;
		data.bladeBufferLOD1 = blade_buffer_LOD1;

		compute_pass_event eventTime = grass_settings::depthPrepass ? compute_pass_before_depth_prepass : compute_pass_before_opaque;
		compute_pass->addTask<grass_update_pipeline>(eventTime, std::move(data));
	}

	void GrassComponent::render(opaque_render_pass* render_pass)
	{
		grass_render_data data = { settings, draw_buffer, blade_buffer_LOD0, blade_buffer_LOD1, wind_direction, (uint32_t)component_data->entity_handle };
		if (grass_settings::depthPrepass)
		{
			render_pass->renderObject<grass_pipeline, grass_depth_prepass_pipeline>(data, data);
		}
		else
		{
			render_pass->renderObject<grass_no_depth_prepass_pipeline>(data);
		}
	}

}