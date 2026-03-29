// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "rendering/render_utils.h"
#include "rendering/texture_preprocessing.h"
#include "rendering/render_resources.h"
#include "rendering/render_algorithms.h"
#include "rendering/outline.h"
#include "rendering/pbr.h"
#include "rendering/bitonic_sort.h"
#include "rendering/debug_visualization.h"
#include "rendering/light_probe.h"
#include "rendering/raytraced_reflections.h"
#include "rendering/pbr_raytracer.h"
#include "rendering/shadow_map.h"

#include "core/random.h"
#include "core/job_system.h"

#include "dx/dx_context.h"

#include "animation/skinning.h"

#include "particles/particles.h"
#include "particles/particle_systems.h"

#include "terrain/terrain.h"
#include "terrain/proc_placement.h"
#include "terrain/grass.h"
#include "terrain/water.h"
#include "terrain/tree.h"

namespace era_engine
{
	// Defined in render_algorithms.cpp.
	void loadCommonShaders();

	static vec2 haltonSequence[128];

	void initializeRenderUtils()
	{
		initializeTexturePreprocessing();
		initializeDepthPrepassPipelines();
		initializeOutlinePipelines();
		animation::initializeSkinning();
		initializeShadowPipelines();
		initializeTerrainPipelines();
		initializeLightProbePipelines();
		initializeRTReflectionsPipelines();
		initializeProceduralPlacementPipelines();
		initializeGrassPipelines();
		initializeWaterPipelines();
		initializeTreePipelines();
		loadCommonShaders();

		debug_simple_pipeline::initialize();
		debug_unlit_pipeline::initialize();
		debug_unlit_line_pipeline::initialize();
		pbr_pipeline::initialize();
		particle_system::initializePipeline();
		initializeBitonicSort();
		loadAllParticleSystemPipelines();

		createAllPendingReloadablePipelines();
		render_resources::initializeGlobalResources();

		for (uint32 i = 0; i < arraysize(haltonSequence); ++i)
		{
			haltonSequence[i] = halton23(i) * 2.f - vec2(1.f);
		}
	}

	void endFrameCommon()
	{
		checkForChangedPipelines();
	}

	void buildCameraConstantBuffer(const render_camera& camera, float cameraJitterStrength, camera_cb& outCB)
	{
		if (cameraJitterStrength > 0.f)
		{
			vec2 jitter = haltonSequence[dxContext.frameID % arraysize(haltonSequence)] / vec2((float)camera.width, (float)camera.height) * cameraJitterStrength;
			buildCameraConstantBuffer(camera.getJitteredVersion(jitter), jitter, outCB);
		}
		else
		{
			buildCameraConstantBuffer(camera, vec2(0.f, 0.f), outCB);
		}
	}

	void buildCameraConstantBuffer(const render_camera& camera, vec2 jitter, camera_cb& outCB)
	{
		outCB.prevFrameView = outCB.view;
		outCB.prevFrameViewProj = outCB.viewProj;
		outCB.viewProj = camera.viewProj;
		outCB.view = camera.view;
		outCB.proj = camera.proj;
		outCB.invViewProj = camera.invViewProj;
		outCB.invView = camera.invView;
		outCB.invProj = camera.invProj;
		outCB.position = vec4(camera.position, 1.f);
		outCB.forward = vec4(camera.rotation * vec3(0.f, 0.f, -1.f), 0.f);
		outCB.right = vec4(camera.rotation * vec3(1.f, 0.f, 0.f), 0.f);
		outCB.up = vec4(camera.rotation * vec3(0.f, 1.f, 0.f), 0.f);
		outCB.projectionParams = vec4(camera.nearPlane, camera.farPlane, camera.farPlane / camera.nearPlane, 1.f - camera.farPlane / camera.nearPlane);
		outCB.viewSpaceTopLeftFrustumVector = vec4(camera.restoreViewSpacePosition(vec2(0.f, 0.f), 0.f) / camera.nearPlane, 0.f);
		outCB.extentAtDistanceOne = abs(camera.restoreViewSpacePosition(vec2(1.f, 1.f), 0.f).xy / camera.nearPlane - outCB.viewSpaceTopLeftFrustumVector.xy);
		outCB.screenDims = vec2((float)camera.width, (float)camera.height);
		outCB.invScreenDims = vec2(1.f / camera.width, 1.f / camera.height);
		outCB.prevFrameJitter = outCB.jitter;
		outCB.jitter = jitter;
	}

	raytracing_object_type defineBlasFromMesh(const ref<multi_mesh>& mesh)
	{
		if (dxContext.featureSupport.raytracing())
		{
			raytracing_blas_builder blasBuilder;
			std::vector<ref<pbr_material>> raytracingMaterials;

			for (auto& sm : mesh->submeshes)
			{
				blasBuilder.push(mesh->mesh.vertexBuffer, mesh->mesh.indexBuffer, sm.info);
				raytracingMaterials.push_back(sm.material);
			}

			ref<raytracing_blas> blas = blasBuilder.finish();
			raytracing_object_type type = pbr_raytracer::defineObjectType(blas, raytracingMaterials);
			return type;
		}
		else
		{
			return {};
		}
	}

	void addRaytracingComponentAsync(Entity entity, ref<multi_mesh> mesh)
	{
		struct add_ray_tracing_data
		{
			Entity entity;
			ref<multi_mesh> mesh;
		};

		add_ray_tracing_data data = { entity, mesh };

		low_priority_job_queue.createJob<add_ray_tracing_data>([](add_ray_tracing_data& data, JobHandle)
			{
				data.mesh->loadJob.wait_for_completion();

				struct create_component_data
				{
					Entity entity;
					raytracing_object_type blas;
				};

				create_component_data createData = { data.entity, defineBlasFromMesh(data.mesh) };

				main_thread_job_queue.createJob<create_component_data>([](create_component_data& data, JobHandle)
					{
						data.entity.add_component<RaytraceComponent>(data.blas);
					}, createData).submit_now();

			}, data).submit_now();
	}
}