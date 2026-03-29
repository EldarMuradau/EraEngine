// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "proc_placement.h"

#include "dx/dx_command_list.h"
#include "dx/dx_pipeline.h"
#include "dx/dx_barrier_batcher.h"
#include "dx/dx_profiling.h"

#include "geometry/mesh_builder.h"

#include "rendering/render_utils.h"
#include "rendering/material.h"
#include "rendering/render_pass.h"
#include "rendering/pbr.h"

#include <rttr/registration>

#include "proc_placement_rs.hlsli"

namespace era_engine
{
	static dx_pipeline generatePointsPipeline;
	static dx_pipeline prefixSumPipeline;
	static dx_pipeline createDrawCallsPipeline;
	static dx_pipeline createTransformsPipeline;

	static dx_pipeline visualizePointsPipeline;

	static dx_mesh visualizePointsMesh;
	static submesh_info visualizePointsSubmesh;
	static dx_command_signature visualizePointsCommandSignature;

	void initializeProceduralPlacementPipelines()
	{
		generatePointsPipeline = createReloadablePipeline("proc_placement_generate_points_cs");
		prefixSumPipeline = createReloadablePipeline("proc_placement_prefix_sum_cs");
		createDrawCallsPipeline = createReloadablePipeline("proc_placement_create_draw_calls_cs");
		createTransformsPipeline = createReloadablePipeline("proc_placement_create_transforms_cs");

		{
			auto& desc = CREATE_GRAPHICS_PIPELINE
				.inputLayout(inputLayout_position_uv_normal_tangent)
				.renderTargets(ldrFormat, depthStencilFormat);

			visualizePointsPipeline = createReloadablePipeline(desc, { "proc_placement_points_vs", "proc_placement_points_ps" }, rs_in_pixel_shader, true);
		}


		mesh_builder builder(mesh_creation_flags_with_positions);
		//builder.pushSphere({});
		builder.pushArrow({});
		visualizePointsSubmesh = builder.endSubmesh();
		visualizePointsMesh = builder.createDXMesh();


		D3D12_INDIRECT_ARGUMENT_DESC argumentDescs[2] = {};
		argumentDescs[0].Type = D3D12_INDIRECT_ARGUMENT_TYPE_SHADER_RESOURCE_VIEW;
		argumentDescs[0].ShaderResourceView.RootParameterIndex = 1;
		argumentDescs[1].Type = D3D12_INDIRECT_ARGUMENT_TYPE_DRAW_INDEXED;
		visualizePointsCommandSignature = createCommandSignature(*visualizePointsPipeline.rootSignature, argumentDescs, arraysize(argumentDescs), sizeof(placement_draw));
	}

	struct render_proc_placement_layer_data
	{
		ref<dx_buffer> commandBuffer;
		uint32 commandBufferOffset;

		dx_vertex_buffer_group_view vertexBuffer;
		dx_index_buffer_view indexBuffer;

		ref<dx_texture> albedo;
	};

	struct render_proc_placement_layer_pipeline
	{
		PIPELINE_SETUP_DECL;
		PIPELINE_RENDER_DECL(render_proc_placement_layer_data);
	};

	PIPELINE_SETUP_IMPL(render_proc_placement_layer_pipeline)
	{
		cl->setPipelineState(*visualizePointsPipeline.pipeline);
		cl->setGraphicsRootSignature(*visualizePointsPipeline.rootSignature);
		cl->setPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
		cl->setGraphicsDynamicConstantBuffer(0, common.cameraCBV);
	}

	PIPELINE_RENDER_IMPL(render_proc_placement_layer_pipeline, render_proc_placement_layer_data)
	{
		cl->setVertexBuffer(0, data.vertexBuffer.positions);
		cl->setVertexBuffer(1, data.vertexBuffer.others);
		cl->setIndexBuffer(data.indexBuffer);
		cl->setDescriptorHeapSRV(2, 0, data.albedo);
		cl->drawIndirect(visualizePointsCommandSignature, 1, data.commandBuffer, data.commandBufferOffset * sizeof(placement_draw));
	}

#define READBACK 0

#if READBACK
	static ref<dx_buffer> readbackIndirect;
	static ref<dx_buffer> readbackMeshCounts;
	static ref<dx_buffer> readbackMeshOffsets;
#endif

	RTTR_REGISTRATION
	{
		using namespace rttr;
		rttr::registration::class_<ProcPlacementComponent>("ProcPlacementComponent")
			.constructor<>()
			.constructor<ref<Entity::EcsData>, const std::vector<proc_placement_layer_desc>&>()
			.property("layers", &ProcPlacementComponent::layers);
	}

	ProcPlacementComponent::ProcPlacementComponent(ref<Entity::EcsData> _data, const std::vector<proc_placement_layer_desc>& _layers)
		: Component(_data)
	{
		std::vector<placement_draw> drawArgs;

		{
			placement_draw draw;
			draw.draw.BaseVertexLocation = visualizePointsSubmesh.baseVertex;
			draw.draw.IndexCountPerInstance = visualizePointsSubmesh.numIndices;
			draw.draw.StartIndexLocation = visualizePointsSubmesh.firstIndex;
			draw.draw.InstanceCount = 0;
			draw.draw.StartInstanceLocation = 0;
			drawArgs.push_back(draw);

			submesh_to_mesh.push_back(0);
		}

		uint32 globalMeshOffset = 1;

		for (const auto& layerDesc : layers)
		{
			PlacementLayer layer;
			layer.name = layerDesc.name;
			layer.footprint = layerDesc.footprint;
			layer.global_mesh_offset = globalMeshOffset;
			layer.num_meshes = 0;

			for (uint32 i = 0; i < 4; ++i)
			{
				const ref<multi_mesh>& mesh = layerDesc.meshes[i];
				layer.meshes[i] = mesh;

				if (mesh)
				{
					++layer.num_meshes;

					for (const auto& sub : mesh->submeshes)
					{
						placement_draw draw;
						draw.draw.BaseVertexLocation = sub.info.baseVertex;
						draw.draw.IndexCountPerInstance = sub.info.numIndices;
						draw.draw.StartIndexLocation = sub.info.firstIndex;
						draw.draw.InstanceCount = 0;
						draw.draw.StartInstanceLocation = 0;
						drawArgs.push_back(draw);

						submesh_to_mesh.push_back(globalMeshOffset + i);

						has_valid_meshes = true;
					}
				}
			}

			globalMeshOffset += layer.num_meshes;

			this->layers.push_back(layer);
		}

		ASSERT(globalMeshOffset <= 512); // Prefix sum currently only supports up to 512 points.

		placement_point_buffer = createBuffer(sizeof(placement_point), 100000, 0, true);
		transform_buffer = createBuffer(sizeof(placement_transform), 100000, 0, true);
		mesh_count_buffer = createBuffer(sizeof(uint32), globalMeshOffset, 0, true, true);
		mesh_offset_buffer = createBuffer(sizeof(uint32), globalMeshOffset, 0, true, true);
		submesh_to_mesh_buffer = createBuffer(sizeof(uint32), (uint32)submesh_to_mesh.size(), submesh_to_mesh.data());
		draw_indirect_buffer = createBuffer(sizeof(placement_draw), (uint32)drawArgs.size(), drawArgs.data(), true);


#if READBACK
		readbackIndirect = createReadbackBuffer(sizeof(placement_draw), (uint32)drawArgs.size());
		readbackMeshCounts = createReadbackBuffer(sizeof(uint32), globalMeshOffset);
		readbackMeshOffsets = createReadbackBuffer(sizeof(uint32), globalMeshOffset);
#endif
	}

	ProcPlacementComponent::~ProcPlacementComponent()
	{
	}

	void ProcPlacementComponent::generate(const render_camera& camera, const TerrainComponent& terrain, const vec3& position_offset)
	{
		if (!has_valid_meshes)
		{
			return;
		}

		dx_command_list* cl = dxContext.getFreeComputeCommandList(false);

		{
			PROFILE_ALL(cl, "Procedural placement");

			{
				PROFILE_ALL(cl, "Generate points");

				camera_frustum_planes frustum = camera.getWorldSpaceFrustumPlanes();

				const float radiusInUVSpace = sqrt(1.f / (2.f * sqrt(3.f) * arraysize(POISSON_SAMPLES)));
				float diameterInUVSpace = radiusInUVSpace * 2.f;
				float diameterInWorldSpace = diameterInUVSpace * terrain.chunkSize;

				cl->setPipelineState(*generatePointsPipeline.pipeline);
				cl->setComputeRootSignature(*generatePointsPipeline.rootSignature);

				vec3 minCorner = terrain.get_min_corner(position_offset);
				vec3 chunkSize(terrain.chunkSize, terrain.amplitudeScale, terrain.chunkSize);

				cl->clearUAV(mesh_count_buffer, 0u);

				for (uint32 z = 0; z < terrain.chunksPerDim; ++z)
				{
					for (uint32 x = 0; x < terrain.chunksPerDim; ++x)
					{
						auto& chunk = terrain.chunk(x, z);
						vec3 chunkMinCorner = minCorner + vec3(x * terrain.chunkSize, 0.f, z * terrain.chunkSize);
						vec3 chunkMaxCorner = chunkMinCorner + chunkSize;

						bounding_box aabb = { chunkMinCorner, chunkMaxCorner };
						if (!frustum.cullWorldSpaceAABB(aabb))
						{
							cl->setDescriptorHeapSRV(PROC_PLACEMENT_GENERATE_POINTS_RS_RESOURCES, 0, chunk.heightmap);
							cl->setDescriptorHeapSRV(PROC_PLACEMENT_GENERATE_POINTS_RS_RESOURCES, 1, chunk.normalmap);
							cl->setDescriptorHeapUAV(PROC_PLACEMENT_GENERATE_POINTS_RS_RESOURCES, 2, placement_point_buffer);
							cl->setDescriptorHeapUAV(PROC_PLACEMENT_GENERATE_POINTS_RS_RESOURCES, 3, mesh_count_buffer);


							proc_placement_generate_points_cb cb;
							cb.amplitudeScale = terrain.amplitudeScale;
							cb.chunkCorner = chunkMinCorner;
							cb.chunkSize = terrain.chunkSize;

							for (const auto& layer : layers)
							{
								float footprint = layer.footprint;

								float scaling = diameterInWorldSpace / footprint;
								uint32 numGroupsPerDim = (uint32)ceil(scaling);

								cb.densities = vec4(0.f);
								for (uint32 i = 0; i < layer.num_meshes; ++i)
								{
									cb.densities.data[i] = layer.densities[i];
								}

								cb.uvScale = 1.f / scaling;
								cb.numMeshes = layer.num_meshes;
								cb.globalMeshOffset = layer.global_mesh_offset;

								cl->setCompute32BitConstants(PROC_PLACEMENT_GENERATE_POINTS_RS_CB, cb);

								cl->dispatch(numGroupsPerDim, numGroupsPerDim, 1);

								barrier_batcher(cl)
									.uav(placement_point_buffer)
									.uav(mesh_count_buffer);
							}
						}
					}
				}
			}

			barrier_batcher(cl)
				.transition(mesh_count_buffer, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_GENERIC_READ)
				.transition(placement_point_buffer, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_GENERIC_READ);

			{
				PROFILE_ALL(cl, "Prefix sum");

				cl->setPipelineState(*prefixSumPipeline.pipeline);
				cl->setComputeRootSignature(*prefixSumPipeline.rootSignature);

				cl->setCompute32BitConstants(PROC_PLACEMENT_PREFIX_SUM_RS_CB, prefix_sum_cb{ mesh_count_buffer->elementCount - 1 });

				// Offset by 1 element, since that is the point count.
				cl->setRootComputeUAV(PROC_PLACEMENT_PREFIX_SUM_RS_OUTPUT, mesh_offset_buffer->gpuVirtualAddress + mesh_offset_buffer->elementSize);
				cl->setRootComputeSRV(PROC_PLACEMENT_PREFIX_SUM_RS_INPUT, mesh_count_buffer->gpuVirtualAddress + mesh_count_buffer->elementSize);

				cl->dispatch(1);
			}

			barrier_batcher(cl)
				.transition(mesh_offset_buffer, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_GENERIC_READ);

			{
				PROFILE_ALL(cl, "Create draw calls");

				cl->setPipelineState(*createDrawCallsPipeline.pipeline);
				cl->setComputeRootSignature(*createDrawCallsPipeline.rootSignature);

				proc_placement_create_draw_calls_cb cb;
				cb.transformAddressHigh = (uint32)(transform_buffer->gpuVirtualAddress >> 32);
				cb.transformAddressLow = (uint32)(transform_buffer->gpuVirtualAddress & 0xFFFFFFFF);
				cb.stride = transform_buffer->elementSize;
				cl->setCompute32BitConstants(PROC_PLACEMENT_CREATE_DRAW_CALLS_RS_CB, cb);
				cl->setRootComputeUAV(PROC_PLACEMENT_CREATE_DRAW_CALLS_RS_OUTPUT, draw_indirect_buffer);
				cl->setRootComputeSRV(PROC_PLACEMENT_CREATE_DRAW_CALLS_RS_MESH_COUNTS, mesh_count_buffer);
				cl->setRootComputeSRV(PROC_PLACEMENT_CREATE_DRAW_CALLS_RS_MESH_OFFSETS, mesh_offset_buffer);
				cl->setRootComputeSRV(PROC_PLACEMENT_CREATE_DRAW_CALLS_RS_SUBMESH_TO_MESH, submesh_to_mesh_buffer);

				cl->dispatch(bucketize(draw_indirect_buffer->elementCount, PROC_PLACEMENT_CREATE_DRAW_CALLS_BLOCK_SIZE));
			}

#if READBACK
			cl->copyBufferRegionToBuffer(meshCountBuffer, readbackMeshCounts, 0, meshCountBuffer->elementCount, 0);
#endif

			barrier_batcher(cl)
				.transition(mesh_count_buffer, D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
				.uav(draw_indirect_buffer);

			{
				PROFILE_ALL(cl, "Create transforms");

				cl->setPipelineState(*createTransformsPipeline.pipeline);
				cl->setComputeRootSignature(*createTransformsPipeline.rootSignature);

				cl->setRootComputeUAV(PROC_PLACEMENT_CREATE_TRANSFORMS_RS_TRANSFORMS, transform_buffer);
				cl->setRootComputeUAV(PROC_PLACEMENT_CREATE_TRANSFORMS_RS_MESH_COUNTS, mesh_count_buffer);
				cl->setRootComputeSRV(PROC_PLACEMENT_CREATE_TRANSFORMS_RS_POINTS, placement_point_buffer);
				cl->setRootComputeSRV(PROC_PLACEMENT_CREATE_TRANSFORMS_RS_MESH_OFFSETS, mesh_offset_buffer);

				cl->dispatch(bucketize(placement_point_buffer->elementCount, PROC_PLACEMENT_CREATE_TRANSFORMS_BLOCK_SIZE)); // TODO
				cl->uavBarrier(transform_buffer);
				cl->uavBarrier(mesh_count_buffer);
			}
	}

#if READBACK
		cl->transitionBarrier(draw_indirect_buffer, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_GENERIC_READ);
		cl->copyBufferRegionToBuffer(draw_indirect_buffer, readbackIndirect, 0, draw_indirect_buffer->elementCount, 0);
		cl->copyBufferRegionToBuffer(mesh_offset_buffer, readbackMeshOffsets, 0, mesh_offset_buffer->elementCount, 0);
#endif

		dxContext.executeCommandList(cl);

#if READBACK
		dxContext.flushApplication();

		{
			placement_draw* draw = (placement_draw*)mapBuffer(readbackIndirect, true);
			uint32* offsets = (uint32*)mapBuffer(readbackMeshOffsets, true);
			uint32* counts = (uint32*)mapBuffer(readbackMeshCounts, true);

			for (uint32 i = 0; i < readbackIndirect->elementCount; ++i)
			{
				uint32 meshID = submeshToMesh[i];

				D3D12_GPU_VIRTUAL_ADDRESS expectedAddress = transformBuffer->gpuVirtualAddress + offsets[meshID] * transformBuffer->elementSize;
				uint32 expectedHigh = (uint32)(expectedAddress >> 32);
				uint32 expectedLow = (uint32)(expectedAddress & 0xFFFFFFFF);

				ASSERT(draw[i].transformSRVHigh == expectedHigh);
				ASSERT(draw[i].transformSRVLow == expectedLow);
			}

			LOG_MESSAGE("%u vs %u (%u), %u vs %u (%u), %u vs %u (%u)",
				draw[0].draw.InstanceCount, counts[0], offsets[0],
				draw[1].draw.InstanceCount, counts[1], offsets[1],
				draw[2].draw.InstanceCount, counts[2], offsets[2]);

			unmapBuffer(readbackMeshCounts, false);
			unmapBuffer(readbackMeshOffsets, false);
			unmapBuffer(readbackIndirect, false);
		}
#endif
	}

	void ProcPlacementComponent::render(ldr_render_pass* render_pass)
	{
#if 0

		render_proc_placement_layer_data data = { transform_buffer, draw_indirect_buffer, 0, visualize_points_mesh.vertexBuffer, visualize_points_mesh.indexBuffer };
		render_pass->renderObject<render_proc_placement_layer_pipeline>(data);

#else

		uint32 drawCallOffset = 1;

		for (const auto& layer : layers)
		{
			for (uint32 i = 0; i < layer.num_meshes; ++i)
			{
				for (uint32 j = 0; j < (uint32)layer.meshes[i]->submeshes.size(); ++j)
				{
					// TODO: We could pack the vertex and index buffer into the indirect call, but I'm not sure if that fits well with stuff like procedural objects.
					render_proc_placement_layer_data data = { draw_indirect_buffer, drawCallOffset,
						layer.meshes[i]->mesh.vertexBuffer, layer.meshes[i]->mesh.indexBuffer, layer.meshes[i]->submeshes[j].material->albedo };

					render_pass->renderObject<render_proc_placement_layer_pipeline>(data);

					++drawCallOffset;
				}
			}
		}

#endif
	}

}