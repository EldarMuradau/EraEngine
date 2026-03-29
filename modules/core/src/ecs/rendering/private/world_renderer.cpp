#include "ecs/rendering/world_renderer.h"
#include "ecs/rendering/mesh_component.h"
#include "ecs/base_components/base_components.h"

#include "core/cpu_profiling.h"
#include "core/string.h"

#include "rendering/pbr.h"
#include "rendering/depth_prepass.h"
#include "rendering/outline.h"
#include "rendering/shadow_map.h"

#include "asset/pbr_material_desc.h"

#include "geometry/mesh.h"

#include "dx/dx_context.h"

#include "terrain/tree.h"
#include "terrain/water.h"
#include "terrain/terrain.h"
#include "terrain/proc_placement.h"
#include "terrain/grass.h"

namespace era_engine
{

	static bool shouldRender(bounding_sphere s, bounding_box aabb, const trs& transform)
	{
		s.center = conjugate(transform.rotation) * (s.center - transform.position) + transform.position;
		aabb.minCorner *= transform.scale;
		aabb.maxCorner *= transform.scale;
		return aabb.contains(s.center) || sphereVsAABB(s, aabb);
	}

	static bool shouldRender(const camera_frustum_planes& frustum, const MeshComponent& mesh, const TransformComponent& transform)
	{
		return mesh.mesh && !mesh.is_hidden && (mesh.mesh->loadState.load() == AssetLoadState::LOADED) && ((mesh.mesh->aabb.maxCorner.x == mesh.mesh->aabb.minCorner.x) || !frustum.cullModelSpaceAABB(mesh.mesh->aabb, transform.get_world_transform()));
	}

	static bool shouldRender(const bounding_sphere& frustum, const MeshComponent& mesh, const TransformComponent& transform)
	{
		return mesh.mesh && !mesh.is_hidden && (mesh.mesh->loadState.load() == AssetLoadState::LOADED) && ((mesh.mesh->aabb.maxCorner.x == mesh.mesh->aabb.minCorner.x) || shouldRender(frustum, mesh.mesh->aabb, transform.get_world_transform()));
	}

	static bool shouldRender(const light_frustum& frustum, const MeshComponent& mesh, const TransformComponent& transform)
	{
		return (frustum.type == light_frustum_standard) ? shouldRender(frustum.frustum, mesh, transform) : shouldRender(frustum.sphere, mesh, transform);
	}

	template <typename group_t>
	std::unordered_map<multi_mesh*, offset_count> getOffsetsPerMesh(group_t group)
	{
		uint32 groupSize = (uint32)group.size();

		std::unordered_map<multi_mesh*, offset_count> ocPerMesh;

		uint32 index = 0;
		for (Entity::Handle entityHandle : group)
		{
			MeshComponent& mesh = group.get<MeshComponent>(entityHandle);
			++ocPerMesh[mesh.mesh.get()].count;
		}

		uint32 offset = 0;
		for (auto& [mesh, oc] : ocPerMesh)
		{
			oc.offset = offset;
			offset += oc.count;
			oc.count = 0;
		}

		ocPerMesh.erase(nullptr);

		return ocPerMesh;
	}

	static void addToRenderPass(PbrMaterialShader shader, const pbr_render_data& data, const depth_prepass_data& depthPrepassData,
		opaque_render_pass* opaqueRenderPass, transparent_render_pass* transparentRenderPass)
	{
		switch (shader)
		{
		case pbr_material_shader_default:
		case pbr_material_shader_double_sided:
		{
			if (shader == pbr_material_shader_default)
			{
				opaqueRenderPass->renderObject<pbr_pipeline::opaque>(data);
				opaqueRenderPass->renderDepthOnly<depth_prepass_pipeline::single_sided>(depthPrepassData);
			}
			else
			{
				opaqueRenderPass->renderObject<pbr_pipeline::opaque_double_sided>(data);
				opaqueRenderPass->renderDepthOnly<depth_prepass_pipeline::double_sided>(depthPrepassData);
			}
		} break;
		case pbr_material_shader_alpha_cutout:
		{
			opaqueRenderPass->renderObject<pbr_pipeline::opaque_double_sided>(data);
			opaqueRenderPass->renderDepthOnly<depth_prepass_pipeline::alpha_cutout>(depthPrepassData);
		} break;
		case pbr_material_shader_transparent:
		{
			transparentRenderPass->renderObject<pbr_pipeline::transparent>(data);
		} break;
		}
	}

	static void addToStaticRenderPass(PbrMaterialShader shader, const shadow_render_data& shadowData, shadow_render_pass_base* shadowRenderPass, bool isPointLight)
	{
		switch (shader)
		{
		case pbr_material_shader_default:
		{
			if (isPointLight)
			{
				shadowRenderPass->renderStaticObject<point_shadow_pipeline::single_sided>(shadowData);
			}
			else
			{
				shadowRenderPass->renderStaticObject<shadow_pipeline::single_sided>(shadowData);
			}
		} break;
		case pbr_material_shader_double_sided:
		case pbr_material_shader_alpha_cutout:
		{
			if (isPointLight)
			{
				shadowRenderPass->renderStaticObject<point_shadow_pipeline::double_sided>(shadowData);
			}
			else
			{
				shadowRenderPass->renderStaticObject<shadow_pipeline::double_sided>(shadowData);
			}
		} break;
		case pbr_material_shader_transparent:
		{
			// Do nothing
		} break;
		}
	}

	static void addToDynamicRenderPass(PbrMaterialShader shader, const shadow_render_data& shadowData, shadow_render_pass_base* shadowRenderPass, bool isPointLight)
	{
		switch (shader)
		{
		case pbr_material_shader_default:
		{
			if (isPointLight)
			{
				shadowRenderPass->renderDynamicObject<point_shadow_pipeline::single_sided>(shadowData);
			}
			else
			{
				shadowRenderPass->renderDynamicObject<shadow_pipeline::single_sided>(shadowData);
			}
		} break;
		case pbr_material_shader_double_sided:
		case pbr_material_shader_alpha_cutout:
		{
			if (isPointLight)
			{
				shadowRenderPass->renderDynamicObject<point_shadow_pipeline::double_sided>(shadowData);
			}
			else
			{
				shadowRenderPass->renderDynamicObject<shadow_pipeline::double_sided>(shadowData);
			}
		} break;
		case pbr_material_shader_transparent:
		{
			// Do nothing
		} break;
		}
	}

	template <typename group_t>
	static void renderStaticObjectsToMainCamera(group_t group, std::unordered_map<multi_mesh*, offset_count> ocPerMesh,
		const camera_frustum_planes& frustum, Allocator& arena, Entity::Handle selectedObjectID,
		opaque_render_pass* opaqueRenderPass, transparent_render_pass* transparentRenderPass, ldr_render_pass* ldrRenderPass)
	{
		uint32 groupSize = (uint32)group.size();

		dx_allocation transformAllocation = dxContext.allocateDynamicBuffer(groupSize * sizeof(mat4), 4);
		mat4* transforms = (mat4*)transformAllocation.cpuPtr;

		dx_allocation objectIDAllocation = dxContext.allocateDynamicBuffer(groupSize * sizeof(uint32), 4);
		uint32* objectIDs = (uint32*)objectIDAllocation.cpuPtr;

		for (auto [entityHandle, transform, mesh] : group.each())
		{
			if (!shouldRender(frustum, mesh, transform))
				continue;

			if (transform.type == TransformComponent::DYNAMIC)
				continue;

			const dx_mesh& dxMesh = mesh.mesh->mesh;

			offset_count& oc = ocPerMesh.at(mesh.mesh.get());

			uint32 index = oc.offset + oc.count;
			const trs& world_transform = transform.get_world_transform();

			transforms[index] = trs_to_mat4(world_transform);
			objectIDs[index] = (uint32)entityHandle;

			++oc.count;

			if (entityHandle == selectedObjectID)
			{
				for (auto& sm : mesh.mesh->submeshes)
				{
					renderOutline(ldrRenderPass, transforms[index], dxMesh.vertexBuffer, dxMesh.indexBuffer, sm.info);
				}
			}
		}

		D3D12_GPU_VIRTUAL_ADDRESS transformsAddress = transformAllocation.gpuPtr;
		D3D12_GPU_VIRTUAL_ADDRESS objectIDAddress = objectIDAllocation.gpuPtr;

		uint32 numDrawCalls = 0;

		for (auto& [mesh, oc] : ocPerMesh)
		{
			if (oc.count == 0)
				continue;

			D3D12_GPU_VIRTUAL_ADDRESS baseM = transformsAddress + (oc.offset * sizeof(mat4));
			D3D12_GPU_VIRTUAL_ADDRESS baseObjectID = objectIDAddress + (oc.offset * sizeof(uint32));

			const dx_mesh& dxMesh = mesh->mesh;

			if (!dxMesh.indexBuffer || !dxMesh.vertexBuffer.positions)
				continue;

			pbr_render_data data;
			data.transformPtr = baseM;
			data.vertexBuffer = dxMesh.vertexBuffer;
			data.indexBuffer = dxMesh.indexBuffer;
			data.numInstances = oc.count;

			depth_prepass_data depthPrepassData;
			depthPrepassData.transformPtr = baseM;
			depthPrepassData.prevFrameTransformPtr = baseM;
			depthPrepassData.objectIDPtr = baseObjectID;
			depthPrepassData.vertexBuffer = dxMesh.vertexBuffer;
			depthPrepassData.prevFrameVertexBuffer = dxMesh.vertexBuffer.positions;
			depthPrepassData.indexBuffer = dxMesh.indexBuffer;
			depthPrepassData.numInstances = oc.count;

			for (auto& sm : mesh->submeshes)
			{
				data.submesh = sm.info;
				data.material = sm.material;

				depthPrepassData.submesh = data.submesh;
				depthPrepassData.alphaCutoutTextureSRV = (sm.material && sm.material->albedo) ? sm.material->albedo->defaultSRV : dx_cpu_descriptor_handle{};

				addToRenderPass(sm.material->shader, data, depthPrepassData, opaqueRenderPass, transparentRenderPass);

				++numDrawCalls;
			}
		}

		CPU_PROFILE_STAT("Static draw calls", numDrawCalls);
	}

	template <typename group_t>
	static void renderStaticObjectsToShadowMap(group_t group, std::unordered_map<multi_mesh*, offset_count> ocPerMesh,
		const light_frustum& frustum, Allocator& arena, shadow_render_pass_base* shadowRenderPass)
	{
		uint32 groupSize = (uint32)group.size();

		dx_allocation transformAllocation = dxContext.allocateDynamicBuffer(groupSize * sizeof(mat4), 4);
		mat4* transforms = (mat4*)transformAllocation.cpuPtr;

		for (auto [entityHandle, transform, mesh] : group.each())
		{
			if (!shouldRender(frustum, mesh, transform))
				continue;

			if (transform.type == TransformComponent::DYNAMIC)
				continue;

			const dx_mesh& dxMesh = mesh.mesh->mesh;

			offset_count& oc = ocPerMesh.at(mesh.mesh.get());

			uint32 index = oc.offset + oc.count;
			const trs& world_transform = transform.get_world_transform();

			transforms[index] = trs_to_mat4(world_transform);

			++oc.count;
		}

		D3D12_GPU_VIRTUAL_ADDRESS transformsAddress = transformAllocation.gpuPtr;

		for (auto& [mesh, oc] : ocPerMesh)
		{
			if (oc.count == 0)
				continue;

			D3D12_GPU_VIRTUAL_ADDRESS baseM = transformsAddress + (oc.offset * sizeof(mat4));

			const dx_mesh& dxMesh = mesh->mesh;

			if (!dxMesh.indexBuffer || !dxMesh.vertexBuffer.positions)
				continue;

			shadow_render_data data;
			data.transformPtr = baseM;
			data.vertexBuffer = dxMesh.vertexBuffer.positions;
			data.indexBuffer = dxMesh.indexBuffer;
			data.numInstances = oc.count;

			for (auto& sm : mesh->submeshes)
			{
				data.submesh = sm.info;
				addToStaticRenderPass(sm.material->shader, data, shadowRenderPass, frustum.type == light_frustum_sphere);
			}
		}
	}

	static void renderStaticObjects(World* world, const camera_frustum_planes& frustum, Allocator& arena, Entity::Handle selectedObjectID,
		opaque_render_pass* opaqueRenderPass, transparent_render_pass* transparentRenderPass, ldr_render_pass* ldrRenderPass, shadow_passes& shadow)
	{
		CPU_PROFILE_BLOCK("Static objects");

		using specialized_components = ComponentsGroup<
			animation::AnimationComponent,
			TreeComponent
		>;

		auto group = world->group(
			components_group<TransformComponent, MeshComponent>,
			specialized_components{});

		std::unordered_map<multi_mesh*, offset_count> ocPerMesh = getOffsetsPerMesh(group);

		renderStaticObjectsToMainCamera(group, ocPerMesh, frustum, arena, selectedObjectID, opaqueRenderPass, transparentRenderPass, ldrRenderPass);

		for (uint32 i = 0; i < shadow.numShadowRenderPasses; ++i)
		{
			auto& pass = shadow.shadowRenderPasses[i];
			renderStaticObjectsToShadowMap(group, ocPerMesh, pass.frustum, arena, pass.pass);
		}
	}

	template <typename group_t>
	static void renderDynamicObjectsToMainCamera(group_t group, std::unordered_map<multi_mesh*, offset_count> ocPerMesh,
		const camera_frustum_planes& frustum, Allocator& arena, Entity::Handle selectedObjectID,
		opaque_render_pass* opaqueRenderPass, transparent_render_pass* transparentRenderPass, ldr_render_pass* ldrRenderPass)
	{
		uint32 groupSize = (uint32)group.size();

		dx_allocation transformAllocation = dxContext.allocateDynamicBuffer(groupSize * sizeof(mat4) * 2, 4);
		mat4* transforms = (mat4*)transformAllocation.cpuPtr;
		mat4* prevFrameTransforms = transforms + groupSize;

		dx_allocation objectIDAllocation = dxContext.allocateDynamicBuffer(groupSize * sizeof(uint32), 4);
		uint32* objectIDs = (uint32*)objectIDAllocation.cpuPtr;

		for (auto [entityHandle, transform, mesh] : group.each())
		{
			if (!shouldRender(frustum, mesh, transform))
				continue;

			if (transform.type != TransformComponent::DYNAMIC)
				continue;

			const dx_mesh& dxMesh = mesh.mesh->mesh;

			offset_count& oc = ocPerMesh.at(mesh.mesh.get());

			uint32 index = oc.offset + oc.count;
			const trs& world_transform = transform.get_world_transform();

			transforms[index] = trs_to_mat4(world_transform);
			prevFrameTransforms[index] = trs_to_mat4(world_transform);
			objectIDs[index] = (uint32)entityHandle;

			++oc.count;

			if (entityHandle == selectedObjectID)
			{
				for (auto& sm : mesh.mesh->submeshes)
				{
					renderOutline(ldrRenderPass, transforms[index], dxMesh.vertexBuffer, dxMesh.indexBuffer, sm.info);
				}
			}
		}

		D3D12_GPU_VIRTUAL_ADDRESS transformsAddress = transformAllocation.gpuPtr;
		D3D12_GPU_VIRTUAL_ADDRESS prevFrameTransformsAddress = transformAllocation.gpuPtr + (groupSize * sizeof(mat4));
		D3D12_GPU_VIRTUAL_ADDRESS objectIDAddress = objectIDAllocation.gpuPtr;

		uint32 numDrawCalls = 0;

		for (auto& [mesh, oc] : ocPerMesh)
		{
			if (oc.count == 0)
				continue;

			D3D12_GPU_VIRTUAL_ADDRESS baseM = transformsAddress + (oc.offset * sizeof(mat4));
			D3D12_GPU_VIRTUAL_ADDRESS prevBaseM = prevFrameTransformsAddress + (oc.offset * sizeof(mat4));
			D3D12_GPU_VIRTUAL_ADDRESS baseObjectID = objectIDAddress + (oc.offset * sizeof(uint32));

			const dx_mesh& dxMesh = mesh->mesh;

			if (!dxMesh.indexBuffer || !dxMesh.vertexBuffer.positions)
				continue;

			pbr_render_data data;
			data.transformPtr = baseM;
			data.vertexBuffer = dxMesh.vertexBuffer;
			data.indexBuffer = dxMesh.indexBuffer;
			data.numInstances = oc.count;

			depth_prepass_data depthPrepassData;
			depthPrepassData.transformPtr = baseM;
			depthPrepassData.prevFrameTransformPtr = prevBaseM;
			depthPrepassData.objectIDPtr = baseObjectID;
			depthPrepassData.vertexBuffer = dxMesh.vertexBuffer;
			depthPrepassData.prevFrameVertexBuffer = dxMesh.vertexBuffer.positions;
			depthPrepassData.indexBuffer = dxMesh.indexBuffer;
			depthPrepassData.numInstances = oc.count;

			for (auto& sm : mesh->submeshes)
			{
				data.submesh = sm.info;
				data.material = sm.material;

				depthPrepassData.submesh = data.submesh;
				depthPrepassData.alphaCutoutTextureSRV = (sm.material && sm.material->albedo) ? sm.material->albedo->defaultSRV : dx_cpu_descriptor_handle{};

				addToRenderPass(sm.material->shader, data, depthPrepassData, opaqueRenderPass, transparentRenderPass);

				++numDrawCalls;
			}
		}

		CPU_PROFILE_STAT("Dynamic draw calls", numDrawCalls);
	}

	template <typename group_t>
	static void renderDynamicObjectsToShadowMap(group_t group, std::unordered_map<multi_mesh*, offset_count> ocPerMesh,
		const light_frustum& frustum, Allocator& arena, shadow_render_pass_base* shadowRenderPass)
	{
		uint32 groupSize = (uint32)group.size();

		dx_allocation transformAllocation = dxContext.allocateDynamicBuffer(groupSize * sizeof(mat4), 4);
		mat4* transforms = (mat4*)transformAllocation.cpuPtr;

		for (auto [entityHandle, transform, mesh] : group.each())
		{
			if (!shouldRender(frustum, mesh, transform))
				continue;

			if (transform.type != TransformComponent::DYNAMIC)
				continue;

			const dx_mesh& dxMesh = mesh.mesh->mesh;
			if (!ocPerMesh.contains(mesh.mesh.get()))
				continue;
			offset_count& oc = ocPerMesh.at(mesh.mesh.get());

			uint32 index = oc.offset + oc.count;
			const trs& world_transform = transform.get_world_transform();

			transforms[index] = trs_to_mat4(world_transform);

			++oc.count;
		}

		D3D12_GPU_VIRTUAL_ADDRESS transformsAddress = transformAllocation.gpuPtr;

		for (auto& [mesh, oc] : ocPerMesh)
		{
			if (oc.count == 0)
				continue;

			D3D12_GPU_VIRTUAL_ADDRESS baseM = transformsAddress + (oc.offset * sizeof(mat4));

			const dx_mesh& dxMesh = mesh->mesh;

			if (!dxMesh.indexBuffer || !dxMesh.vertexBuffer.positions)
				continue;

			shadow_render_data data;
			data.transformPtr = baseM;
			data.vertexBuffer = dxMesh.vertexBuffer.positions;
			data.indexBuffer = dxMesh.indexBuffer;
			data.numInstances = oc.count;

			for (auto& sm : mesh->submeshes)
			{
				data.submesh = sm.info;
				addToDynamicRenderPass(sm.material->shader, data, shadowRenderPass, frustum.type == light_frustum_sphere);
			}
		}
	}

	static void renderDynamicObjects(World* world, const camera_frustum_planes& frustum, Allocator& arena, Entity::Handle selectedObjectID,
		opaque_render_pass* opaqueRenderPass, transparent_render_pass* transparentRenderPass, ldr_render_pass* ldrRenderPass, shadow_passes& shadow)
	{
		CPU_PROFILE_BLOCK("Dynamic objects");

		auto group = world->group(
			components_group<TransformComponent, MeshComponent>,
			components_group<animation::AnimationComponent>);

		std::unordered_map<multi_mesh*, offset_count> ocPerMesh = getOffsetsPerMesh(group);
		renderDynamicObjectsToMainCamera(group, ocPerMesh, frustum, arena, selectedObjectID, opaqueRenderPass, transparentRenderPass, ldrRenderPass);

		for (uint32 i = 0; i < shadow.numShadowRenderPasses; ++i)
		{
			auto& pass = shadow.shadowRenderPasses[i];
			renderDynamicObjectsToShadowMap(group, ocPerMesh, pass.frustum, arena, pass.pass);
		}
	}

	static void renderAnimatedObjects(World* world, const camera_frustum_planes& frustum, Allocator& arena, Entity::Handle selectedObjectID,
		opaque_render_pass* opaqueRenderPass, transparent_render_pass* transparentRenderPass, ldr_render_pass* ldrRenderPass, shadow_passes& shadow)
	{
		CPU_PROFILE_BLOCK("Animated objects");

		auto group = world->group(
			components_group<TransformComponent, MeshComponent, animation::AnimationComponent>);

		uint32 groupSize = (uint32)group.size();

		dx_allocation transformAllocation = dxContext.allocateDynamicBuffer(groupSize * sizeof(mat4) * 2, 4);
		mat4* transforms = (mat4*)transformAllocation.cpuPtr;
		mat4* prevFrameTransforms = transforms + groupSize;

		dx_allocation objectIDAllocation = dxContext.allocateDynamicBuffer(groupSize * sizeof(uint32), 4);
		uint32* objectIDs = (uint32*)objectIDAllocation.cpuPtr;

		D3D12_GPU_VIRTUAL_ADDRESS transformsAddress = transformAllocation.gpuPtr;
		D3D12_GPU_VIRTUAL_ADDRESS prevFrameTransformsAddress = transformAllocation.gpuPtr + (groupSize * sizeof(mat4));
		D3D12_GPU_VIRTUAL_ADDRESS objectIDAddress = objectIDAllocation.gpuPtr;

		uint32 index = 0;
		for (auto [entityHandle, transform, mesh, anim] : group.each())
		{
			if (!mesh.mesh || mesh.is_hidden || (mesh.mesh->loadState.load() != AssetLoadState::LOADED))
				continue;

			const trs& world_transform = transform.get_world_transform();
			transforms[index] = trs_to_mat4(world_transform);
			prevFrameTransforms[index] = trs_to_mat4(world_transform); //TODO
			objectIDs[index] = (uint32)entityHandle;

			D3D12_GPU_VIRTUAL_ADDRESS baseM = transformsAddress + (index * sizeof(mat4));
			D3D12_GPU_VIRTUAL_ADDRESS prevBaseM = prevFrameTransformsAddress + (index * sizeof(mat4));
			D3D12_GPU_VIRTUAL_ADDRESS baseObjectID = objectIDAddress + (index * sizeof(uint32));

			const dx_mesh& dxMesh = mesh.mesh->mesh;

			pbr_render_data data;
			data.transformPtr = baseM;
			data.vertexBuffer = anim.current_vertex_buffer;
			data.indexBuffer = dxMesh.indexBuffer;
			data.numInstances = 1;

			depth_prepass_data depthPrepassData;
			depthPrepassData.transformPtr = baseM;
			depthPrepassData.prevFrameTransformPtr = prevBaseM;
			depthPrepassData.objectIDPtr = baseObjectID;
			depthPrepassData.vertexBuffer = anim.current_vertex_buffer;
			depthPrepassData.prevFrameVertexBuffer = anim.prev_frame_vertex_buffer.positions ? anim.prev_frame_vertex_buffer.positions : anim.current_vertex_buffer.positions;
			depthPrepassData.indexBuffer = dxMesh.indexBuffer;
			depthPrepassData.numInstances = 1;

			for (auto& sm : mesh.mesh->submeshes)
			{
				data.submesh = sm.info;
				data.material = sm.material;

				data.submesh.baseVertex -= mesh.mesh->submeshes[0].info.baseVertex; // Vertex buffer from skinning already points to first vertex.

				depthPrepassData.submesh = data.submesh;
				depthPrepassData.alphaCutoutTextureSRV = (sm.material && sm.material->albedo) ? sm.material->albedo->defaultSRV : dx_cpu_descriptor_handle{};

				addToRenderPass(sm.material->shader, data, depthPrepassData, opaqueRenderPass, transparentRenderPass);

				shadow_render_data shadowData;
				shadowData.transformPtr = baseM;
				shadowData.vertexBuffer = anim.current_vertex_buffer.positions;
				shadowData.indexBuffer = dxMesh.indexBuffer;
				shadowData.submesh = data.submesh;
				shadowData.numInstances = 1;

				for (uint32 i = 0; i < shadow.numShadowRenderPasses; ++i)
				{
					auto& pass = shadow.shadowRenderPasses[i];
					addToDynamicRenderPass(sm.material->shader, shadowData, pass.pass, pass.frustum.type == light_frustum_sphere);
				}

				if (entityHandle == selectedObjectID)
				{
					renderOutline(ldrRenderPass, transforms[index], anim.current_vertex_buffer, dxMesh.indexBuffer, data.submesh);
				}
			}

			++index;
		}
	}

	static void renderTerrain(const render_camera& camera, World* world, Allocator& arena, Entity::Handle selectedObjectID,
		opaque_render_pass* opaqueRenderPass, transparent_render_pass* transparentRenderPass, ldr_render_pass* ldrRenderPass, sun_shadow_render_pass* sunShadowRenderPass,
		compute_pass* computePass, float dt)
	{
		CPU_PROFILE_BLOCK("Terrain");

		MemoryMarker tempMemoryMarker = arena.get_marker();
		TransformComponent* waterPlaneTransforms = arena.allocate<TransformComponent>(world->number_of_components_of_type<WaterComponent>());
		uint32 numWaterPlanes = 0;

		for (auto [entityHandle, water, transform] : world->group(components_group<WaterComponent, TransformComponent>).each())
		{
			const trs& world_transform = transform.get_world_transform();

			water.render(camera, transparentRenderPass, world_transform.position, vec2(world_transform.scale.x, world_transform.scale.z), dt);

			waterPlaneTransforms[numWaterPlanes++] = transform;
		}

		for (auto [entityHandle, terrain, position] : world->group(components_group<TerrainComponent, TransformComponent>).each())
		{
			const trs& world_transform = position.get_world_transform();

			terrain.render(camera, opaqueRenderPass, sunShadowRenderPass, ldrRenderPass,
				world_transform.position, selectedObjectID == entityHandle, waterPlaneTransforms, numWaterPlanes);
		}
		arena.reset_to_marker(tempMemoryMarker);

		for (auto [entityHandle, terrain, position, placement] : world->group(components_group<TerrainComponent, TransformComponent, ProcPlacementComponent>).each())
		{
			const trs& world_transform = position.get_world_transform();

			placement.generate(camera, terrain, world_transform.position);
			placement.render(ldrRenderPass);
		}

		for (auto [entityHandle, terrain, position, grass] : world->group(components_group<TerrainComponent, TransformComponent, GrassComponent>).each())
		{
			const trs& world_transform = position.get_world_transform();

			grass.generate(computePass, camera, terrain, world_transform.position, dt);
			grass.render(opaqueRenderPass);
		}
	}

	static void renderTrees(World* world, const camera_frustum_planes& frustum, Allocator& arena, Entity::Handle selectedObjectID,
		opaque_render_pass* opaqueRenderPass, transparent_render_pass* transparentRenderPass, ldr_render_pass* ldrRenderPass, sun_shadow_render_pass* sunShadowRenderPass,
		float dt)
	{
		CPU_PROFILE_BLOCK("Trees");

		auto group = world->group(
			components_group<TransformComponent, MeshComponent, TreeComponent>);

		uint32 groupSize = (uint32)group.size();

		std::unordered_map<multi_mesh*, offset_count> ocPerMesh = getOffsetsPerMesh(group);

		dx_allocation transformAllocation = dxContext.allocateDynamicBuffer(groupSize * sizeof(mat4), 4);
		mat4* transforms = (mat4*)transformAllocation.cpuPtr;

		dx_allocation objectIDAllocation = dxContext.allocateDynamicBuffer(groupSize * sizeof(uint32), 4);
		uint32* objectIDs = (uint32*)objectIDAllocation.cpuPtr;

		for (auto [entityHandle, transform, mesh, tree] : group.each())
		{
			if (!shouldRender(frustum, mesh, transform))
				continue;

			const dx_mesh& dxMesh = mesh.mesh->mesh;

			offset_count& oc = ocPerMesh.at(mesh.mesh.get());

			uint32 index = oc.offset + oc.count;

			transforms[index] = trs_to_mat4(transform.get_world_transform());
			objectIDs[index] = (uint32)entityHandle;

			++oc.count;

			if (entityHandle == selectedObjectID)
			{
				for (auto& sm : mesh.mesh->submeshes)
				{
					renderOutline(ldrRenderPass, transforms[index], dxMesh.vertexBuffer, dxMesh.indexBuffer, sm.info);
				}
			}
		}

		D3D12_GPU_VIRTUAL_ADDRESS transformsAddress = transformAllocation.gpuPtr;
		D3D12_GPU_VIRTUAL_ADDRESS objectIDAddress = objectIDAllocation.gpuPtr;

		for (auto& [mesh, oc] : ocPerMesh)
		{
			if (oc.count == 0)
				continue;

			D3D12_GPU_VIRTUAL_ADDRESS baseM = transformsAddress + (oc.offset * sizeof(mat4));
			D3D12_GPU_VIRTUAL_ADDRESS baseObjectID = objectIDAddress + (oc.offset * sizeof(uint32));

			renderTree(opaqueRenderPass, baseM, oc.count, mesh, dt);
		}
	}

	static void renderCloth(World* world, Entity::Handle selectedObjectID,
		opaque_render_pass* opaqueRenderPass, transparent_render_pass* transparentRenderPass, ldr_render_pass* ldrRenderPass, sun_shadow_render_pass* sunShadowRenderPass)
	{
		CPU_PROFILE_BLOCK("Cloth");

		//uint32 groupSize = (uint32)group.size();

		//dx_allocation transformAllocation = dxContext.allocateDynamicBuffer(1 * sizeof(mat4), 4);
		//*(mat4*)transformAllocation.cpuPtr = mat4::identity;

		//dx_allocation objectIDAllocation = dxContext.allocateDynamicBuffer(groupSize * sizeof(uint32), 4);
		//uint32* objectIDs = (uint32*)objectIDAllocation.cpuPtr;

		//D3D12_GPU_VIRTUAL_ADDRESS objectIDAddress = objectIDAllocation.gpuPtr;

		//uint32 index = 0;
		//for (auto [entityHandle, cloth, render] : scene.group<physics::px_cloth_component, physics::px_cloth_render_component>().each())
		//{
		//	pbr_material_desc desc;
		//	// TODO: change it. Temporal solution
		//	desc.albedo = getAssetPath("/resources/assets/Sponza/textures/sponza_curtain_diff.png");
		//	desc.normal = getAssetPath("/resources/assets/Sponza/textures/sponza_fabric_ddn.jpg");
		//	desc.roughness = getAssetPath("/resources/assets/Sponza/textures/sponza_curtain_diff.png");
		//	desc.metallic = "";
		//	desc.shader = pbr_material_shader_double_sided;

		//	static auto clothMaterial = createPBRMaterial(desc);

		//	objectIDs[index] = (uint32)entityHandle;
		//	D3D12_GPU_VIRTUAL_ADDRESS baseObjectID = objectIDAddress + (index * sizeof(uint32));

		//	auto [vb, prevFrameVB, ib, sm] = render.getRenderData(cloth);

		//	pbr_render_data data;
		//	data.transformPtr = transformAllocation.gpuPtr;
		//	data.vertexBuffer = vb;
		//	data.indexBuffer = ib;
		//	data.submesh = sm;
		//	data.material = clothMaterial;
		//	data.numInstances = 1;

		//	depth_prepass_data depthPrepassData;
		//	depthPrepassData.transformPtr = transformAllocation.gpuPtr;
		//	depthPrepassData.prevFrameTransformPtr = transformAllocation.gpuPtr;
		//	depthPrepassData.objectIDPtr = baseObjectID;
		//	depthPrepassData.vertexBuffer = vb;
		//	depthPrepassData.prevFrameVertexBuffer = prevFrameVB.positions ? prevFrameVB.positions : vb.positions;
		//	depthPrepassData.indexBuffer = ib;
		//	depthPrepassData.submesh = sm;
		//	depthPrepassData.numInstances = 1;
		//	depthPrepassData.alphaCutoutTextureSRV = (clothMaterial && clothMaterial->albedo) ? clothMaterial->albedo->defaultSRV : dx_cpu_descriptor_handle{};

		//	addToRenderPass(clothMaterial->shader, data, depthPrepassData, opaqueRenderPass, transparentRenderPass);

		//	if (sunShadowRenderPass)
		//	{
		//		shadow_render_data shadowData;
		//		shadowData.transformPtr = transformAllocation.gpuPtr;
		//		shadowData.vertexBuffer = vb.positions;
		//		shadowData.indexBuffer = ib;
		//		shadowData.numInstances = 1;
		//		shadowData.submesh = data.submesh;

		//		addToDynamicRenderPass(clothMaterial->shader, shadowData, &sunShadowRenderPass->cascades[0], false);
		//	}

		//	if (entityHandle == selectedObjectID)
		//	{
		//		renderOutline(ldrRenderPass, mat4::identity, vb, ib, sm);
		//	}

		//	++index;
		//}
	}

	static void setupSunShadowPass(directional_light& sun, sun_shadow_render_pass* sunShadowRenderPass, bool invalidateShadowMapCache)
	{
		shadow_render_command command = determineSunShadowInfo(sun, invalidateShadowMapCache);
		sunShadowRenderPass->numCascades = sun.numShadowCascades;
		for (uint32 i = 0; i < sun.numShadowCascades; ++i)
		{
			sun_cascade_render_pass& cascadePass = sunShadowRenderPass->cascades[i];
			cascadePass.viewport = command.viewports[i];
			cascadePass.viewProj = sun.viewProjs[i];
		}
		sunShadowRenderPass->copyFromStaticCache = !command.renderStaticGeometry;
	}

	static void setupSpotShadowPasses(World* world, scene_lighting& lighting, bool invalidateShadowMapCache)
	{
		uint32 numSpotLights = world->number_of_components_of_type<SpotLightComponent>();
		if (numSpotLights)
		{
			auto* slPtr = (spot_light_cb*)mapBuffer(lighting.spotLightBuffer, false);
			auto* siPtr = (spot_shadow_info*)mapBuffer(lighting.spotLightShadowInfoBuffer, false);

			for (auto [entityHandle, transform, sl] : world->view<TransformComponent, SpotLightComponent>().each())
			{
				const trs& world_transform = transform.get_world_transform();

				spot_light_cb cb(world_transform.position, world_transform.rotation * vec3(0.f, 0.f, -1.f), sl.color * sl.intensity, sl.innerAngle, sl.outerAngle, sl.distance);

				if (sl.castsShadow && lighting.numSpotShadowRenderPasses < lighting.maxNumSpotShadowRenderPasses)
				{
					cb.shadowInfoIndex = lighting.numSpotShadowRenderPasses++;

					auto [command, si] = determineSpotShadowInfo(cb, (uint32)entityHandle, sl.shadowMapResolution, invalidateShadowMapCache);
					spot_shadow_render_pass& pass = lighting.spotShadowRenderPasses[cb.shadowInfoIndex];

					pass.copyFromStaticCache = !command.renderStaticGeometry;
					pass.viewport = command.viewports[0];
					pass.viewProjMatrix = si.viewProj;

					*siPtr++ = si;
				}

				*slPtr++ = cb;
			}

			unmapBuffer(lighting.spotLightBuffer, true, { 0, numSpotLights });
			unmapBuffer(lighting.spotLightShadowInfoBuffer, true, { 0, lighting.numSpotShadowRenderPasses });
		}
	}

	static void setupPointShadowPasses(World* world, scene_lighting& lighting, bool invalidateShadowMapCache)
	{
		uint32 numPointLights = world->number_of_components_of_type<PointLightComponent>();
		if (numPointLights)
		{
			auto* plPtr = (point_light_cb*)mapBuffer(lighting.pointLightBuffer, false);
			auto* siPtr = (point_shadow_info*)mapBuffer(lighting.pointLightShadowInfoBuffer, false);

			for (auto [entityHandle, position, pl] : world->view<TransformComponent, PointLightComponent>().each())
			{
				const trs& world_transform = position.get_world_transform();

				point_light_cb cb(world_transform.position, pl.color * pl.intensity, pl.radius);

				if (pl.castsShadow && lighting.numPointShadowRenderPasses < lighting.maxNumPointShadowRenderPasses)
				{
					cb.shadowInfoIndex = lighting.numPointShadowRenderPasses++;

					auto [command, si] = determinePointShadowInfo(cb, (uint32)entityHandle, pl.shadowMapResolution, invalidateShadowMapCache);
					point_shadow_render_pass& pass = lighting.pointShadowRenderPasses[cb.shadowInfoIndex];

					pass.copyFromStaticCache0 = !command.renderStaticGeometry;
					pass.copyFromStaticCache1 = !command.renderStaticGeometry;
					pass.viewport0 = command.viewports[0];
					pass.viewport1 = command.viewports[1];
					pass.lightPosition = cb.position;
					pass.maxDistance = cb.radius;

					*siPtr++ = si;
				}

				*plPtr++ = cb;
			}

			unmapBuffer(lighting.pointLightBuffer, true, { 0, numPointLights });
			unmapBuffer(lighting.pointLightShadowInfoBuffer, true, { 0, lighting.numPointShadowRenderPasses });
		}
	}

	void render_world(const render_camera& camera, World* world, Allocator& arena, Entity::Handle selectedObjectID, directional_light& sun, scene_lighting& lighting, bool invalidateShadowMapCache, opaque_render_pass* opaqueRenderPass, transparent_render_pass* transparentRenderPass, ldr_render_pass* ldrRenderPass, sun_shadow_render_pass* sunShadowRenderPass, compute_pass* computePass, float dt)
	{
		CPU_PROFILE_BLOCK("Submit scene render commands");

		setupSunShadowPass(sun, sunShadowRenderPass, invalidateShadowMapCache);
		setupSpotShadowPasses(world, lighting, invalidateShadowMapCache);
		setupPointShadowPasses(world, lighting, invalidateShadowMapCache);

		shadow_passes staticShadowPasses = {};
		shadow_passes dynamicShadowPasses = {};

		MemoryMarker tempMemoryMarker = arena.get_marker();
		uint32 numShadowCastingLights = 1 + lighting.numSpotShadowRenderPasses + lighting.numPointShadowRenderPasses;
		staticShadowPasses.shadowRenderPasses = arena.allocate<shadow_pass>(numShadowCastingLights);
		dynamicShadowPasses.shadowRenderPasses = arena.allocate<shadow_pass>(numShadowCastingLights);

		{
			auto& outPass = dynamicShadowPasses.shadowRenderPasses[dynamicShadowPasses.numShadowRenderPasses++];
			outPass.frustum.frustum = getWorldSpaceFrustumPlanes(sunShadowRenderPass->cascades[sunShadowRenderPass->numCascades - 1].viewProj);
			outPass.frustum.type = light_frustum_standard;
			outPass.pass = &sunShadowRenderPass->cascades[0];

			if (!sunShadowRenderPass->copyFromStaticCache)
			{
				staticShadowPasses.shadowRenderPasses[staticShadowPasses.numShadowRenderPasses++] = outPass;
			}
		}

		for (uint32 i = 0; i < lighting.numSpotShadowRenderPasses; ++i)
		{
			auto pass = &lighting.spotShadowRenderPasses[i];
			auto& outPass = dynamicShadowPasses.shadowRenderPasses[dynamicShadowPasses.numShadowRenderPasses++];
			outPass.frustum.frustum = getWorldSpaceFrustumPlanes(pass->viewProjMatrix);
			outPass.frustum.type = light_frustum_standard;
			outPass.pass = pass;

			if (!pass->copyFromStaticCache)
			{
				staticShadowPasses.shadowRenderPasses[staticShadowPasses.numShadowRenderPasses++] = outPass;
			}
		}

		for (uint32 i = 0; i < lighting.numPointShadowRenderPasses; ++i)
		{
			auto pass = &lighting.pointShadowRenderPasses[i];
			auto& outPass = dynamicShadowPasses.shadowRenderPasses[dynamicShadowPasses.numShadowRenderPasses++];
			outPass.frustum.sphere = { pass->lightPosition, pass->maxDistance };
			outPass.frustum.type = light_frustum_sphere;
			outPass.pass = pass;
			outPass.isPointLight = true;

			if (!pass->copyFromStaticCache0)
			{
				staticShadowPasses.shadowRenderPasses[staticShadowPasses.numShadowRenderPasses++] = outPass;
			}
		}

		bool sunRenderStaticGeometry = !sunShadowRenderPass->copyFromStaticCache;

		camera_frustum_planes frustum = camera.getWorldSpaceFrustumPlanes();

		renderStaticObjects(world, frustum, arena, selectedObjectID, opaqueRenderPass, transparentRenderPass, ldrRenderPass, staticShadowPasses);
		renderDynamicObjects(world, frustum, arena, selectedObjectID, opaqueRenderPass, transparentRenderPass, ldrRenderPass, dynamicShadowPasses);
		renderAnimatedObjects(world, frustum, arena, selectedObjectID, opaqueRenderPass, transparentRenderPass, ldrRenderPass, dynamicShadowPasses);
		renderTerrain(camera, world, arena, selectedObjectID, opaqueRenderPass, transparentRenderPass, ldrRenderPass, sunRenderStaticGeometry ? sunShadowRenderPass : 0,
			computePass, dt);
		renderTrees(world, frustum, arena, selectedObjectID, opaqueRenderPass, transparentRenderPass, ldrRenderPass, sunShadowRenderPass, dt);
		//renderCloth(world, selectedObjectID, opaqueRenderPass, transparentRenderPass, ldrRenderPass, sunShadowRenderPass);

		arena.reset_to_marker(tempMemoryMarker);
	}

}