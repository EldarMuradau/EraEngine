#include "rendering/ecs/private/render_system.h"
#include "rendering/ecs/renderer_holder_root_component.h"

#include "core/imgui.h"
#include "core/log.h"
#include "core/string.h"
#include "core/cpu_profiling.h"
#include "core/ecs/private/input_system.h"
#include "core/ecs/input_receiver_component.h"

#include "rendering/mesh_shader.h"

#include "ecs/update_groups.h"
#include "ecs/base_components/transform_component.h"
#include "ecs/rendering/world_renderer.h"
#include "ecs/rendering/scene_rendering.h"
#include "core/ecs/camera_holder_component.h"
#include "ecs/rendering/mesh_component.h"

#include "audio/audio.h"

#include "terrain/terrain.h"

#include "animation/skinning.h"

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<RenderSystem>("RenderSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("render")))
			.method("begin_frame", &RenderSystem::begin_frame)(metadata("update_group", update_types::INPUT),
				metadata("Before", std::vector<std::string>{"InputSystem::update"}))
			.method("before_render", &RenderSystem::before_render)(metadata("update_group", update_types::BEFORE_RENDER))
			.method("update", &RenderSystem::update)(metadata("update_group", update_types::RENDER))
			.method("after_render", &RenderSystem::after_render)(metadata("update_group", update_types::AFTER_RENDER));
	}

	RenderSystem::RenderSystem(World* _world)
		: System(_world)
	{
		load_custom_shaders();

		renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();
		ASSERT(renderer_holder_rc != nullptr);

		if (dxContext.featureSupport.raytracing())
		{
			raytracingTLAS.initialize();
		}

		renderer_holder_rc->camera.initializeIngame(vec3::zero, quat::identity, deg2rad(70.f), 0.2f);
		renderer_holder_rc->environment.setFromTexture(get_asset_path("/resources/assets/sky/sunset_in_the_chalk_quarry_4k.hdr"));
		renderer_holder_rc->environment.lightProbeGrid.initialize(vec3(-20.f, -1.f, -20.f), vec3(40.f, 20.f, 40.f), 1.5f);

		renderer_holder_rc->sun.direction = normalize(vec3(-0.6f, -1.f, -0.3f));
		renderer_holder_rc->sun.color = vec3(1.f, 0.93f, 0.76f);
		renderer_holder_rc->sun.intensity = 11.1f;
		renderer_holder_rc->sun.numShadowCascades = 3;
		renderer_holder_rc->sun.shadowDimensions = 2048;
		renderer_holder_rc->sun.cascadeDistances = vec4(9.f, 25.f, 50.f, 10000.f);
		renderer_holder_rc->sun.bias = vec4(0.000588f, 0.000784f, 0.000824f, 0.0035f);
		renderer_holder_rc->sun.blendDistances = vec4(5.f, 10.f, 10.f, 10.f);
		renderer_holder_rc->sun.stabilize = true;

		for (uint32 i = 0; i < NUM_BUFFERED_FRAMES; ++i)
		{
			pointLightBuffer[i] = createUploadBuffer(sizeof(point_light_cb), 512, 0);
			spotLightBuffer[i] = createUploadBuffer(sizeof(spot_light_cb), 512, 0);
			decalBuffer[i] = createUploadBuffer(sizeof(pbr_decal_cb), 512, 0);

			spotLightShadowInfoBuffer[i] = createUploadBuffer(sizeof(spot_shadow_info), 512, 0);
			pointLightShadowInfoBuffer[i] = createUploadBuffer(sizeof(point_shadow_info), 512, 0);

			SET_NAME(pointLightBuffer[i]->resource, "Point lights");
			SET_NAME(spotLightBuffer[i]->resource, "Spot lights");
			SET_NAME(decalBuffer[i]->resource, "Decals");

			SET_NAME(spotLightShadowInfoBuffer[i]->resource, "Spot light shadow infos");
			SET_NAME(pointLightShadowInfoBuffer[i]->resource, "Point light shadow infos");
		}

		stackArena.initialize();
	}

	RenderSystem::~RenderSystem()
	{
	}

	void RenderSystem::init()
	{
	}

	void RenderSystem::begin_frame(float dt)
	{
		ZoneScopedN("RenderSystem::begin_frame");

		ImGui::BeginWindowHiddenTabBar("Scene Viewport");
		ImVec2 resolution = ImGui::GetContentRegionAvail();

		if (resolution.x > 0 && resolution.y > 0)
		{
			renderer_holder_rc->width = (uint32)resolution.x;
			renderer_holder_rc->height = (uint32)resolution.y;
		}

		uint32 render_width = renderer_holder_rc->width;
		uint32 render_height = renderer_holder_rc->height;

		main_renderer* renderer = renderer_holder_rc->renderer;

		ImGui::Image(renderer->frameResult, render_width, render_height);
	}

	void RenderSystem::before_render(float dt)
	{
		ZoneScopedN("RenderSystem::before_render");
		main_renderer* renderer = renderer_holder_rc->renderer;

		renderer->beginFrame(renderer_holder_rc->width, renderer_holder_rc->height);

		resetRenderPasses();

		stackArena.reset();
	}

	void RenderSystem::update(float dt)
	{
		ZoneScopedN("RenderSystem::update");
		main_renderer* renderer = renderer_holder_rc->renderer;

		directional_light& sun = renderer_holder_rc->sun;
		pbr_environment& environment = renderer_holder_rc->environment;

		render_camera& camera = renderer_holder_rc->camera;

		environment.update(sun.direction);
		sun.updateMatrices(camera);
		setAudioListener(camera.position, camera.rotation, vec3(0.f));
		environment.lightProbeGrid.visualize(&opaqueRenderPass);

		{
			for (auto [entityHandle, transform, terrain] : world->group(components_group<TransformComponent, TerrainComponent>).each())
			{
				terrain.update();
			}

			scene_lighting lighting;
			lighting.spotLightBuffer = spotLightBuffer[dxContext.bufferedFrameID];
			lighting.pointLightBuffer = pointLightBuffer[dxContext.bufferedFrameID];
			lighting.spotLightShadowInfoBuffer = spotLightShadowInfoBuffer[dxContext.bufferedFrameID];
			lighting.pointLightShadowInfoBuffer = pointLightShadowInfoBuffer[dxContext.bufferedFrameID];
			lighting.spotShadowRenderPasses = spotShadowRenderPasses;
			lighting.pointShadowRenderPasses = pointShadowRenderPasses;
			lighting.maxNumSpotShadowRenderPasses = arraysize(spotShadowRenderPasses);
			lighting.maxNumPointShadowRenderPasses = arraysize(pointShadowRenderPasses);

			render_world(camera, world, stackArena, Entity::NullHandle, sun, lighting, !renderer->settings.cacheShadowMap,
				&opaqueRenderPass, &transparentRenderPass, renderer_holder_rc->ldrRenderPass, &sunShadowRenderPass, &computePass, dt);

			renderer->setSpotLights(spotLightBuffer[dxContext.bufferedFrameID], world->number_of_components_of_type<SpotLightComponent>(), spotLightShadowInfoBuffer[dxContext.bufferedFrameID]);
			renderer->setPointLights(pointLightBuffer[dxContext.bufferedFrameID], world->number_of_components_of_type<PointLightComponent>(), pointLightShadowInfoBuffer[dxContext.bufferedFrameID]);

			if (decals.size())
			{
				updateUploadBufferData(decalBuffer[dxContext.bufferedFrameID], decals.data(), (uint32)(sizeof(pbr_decal_cb) * decals.size()));
				renderer->setDecals(decalBuffer[dxContext.bufferedFrameID], (uint32)decals.size(), decalTexture);
			}

			submitRendererParams(lighting.numSpotShadowRenderPasses, lighting.numPointShadowRenderPasses);
		}
	}

	void RenderSystem::after_render(float dt)
	{
		ZoneScopedN("RenderSystem::after_render");

		main_renderer* renderer = renderer_holder_rc->renderer;

		directional_light& sun = renderer_holder_rc->sun;
		pbr_environment& environment = renderer_holder_rc->environment;

		render_camera& camera = renderer_holder_rc->camera;

		animation::performSkinning(&computePass);

		if (dxContext.featureSupport.raytracing())
		{
			raytracingTLAS.reset();

			for (auto [entityHandle, transform, raytrace] : world->group(components_group<TransformComponent, RaytraceComponent>).each())
			{
				auto handle = raytracingTLAS.instantiate(raytrace.type, transform.get_world_transform());
			}

			renderer->setRaytracingScene(&raytracingTLAS);
		}

		renderer->setEnvironment(environment);
		renderer->setSun(sun);
		renderer->setCamera(camera);

		endFrameCommon();

		const UserInput& input = get_current_frame_input();
		renderer->endFrame(&input);
	}

	void RenderSystem::load_custom_shaders()
	{
		if (dxContext.featureSupport.meshShaders())
		{
			initializeMeshShader();
		}
	}

	void RenderSystem::resetRenderPasses()
	{
		opaqueRenderPass.reset();
		transparentRenderPass.reset();
		renderer_holder_rc->ldrRenderPass->reset();
		sunShadowRenderPass.reset();
		computePass.reset();

		for (uint32 i = 0; i < arraysize(spotShadowRenderPasses); ++i)
		{
			spotShadowRenderPasses[i].reset();
		}

		for (uint32 i = 0; i < arraysize(pointShadowRenderPasses); ++i)
		{
			pointShadowRenderPasses[i].reset();
		}
	}

	void RenderSystem::submitRendererParams(uint32 numSpotLightShadowPasses, uint32 numPointLightShadowPasses)
	{
		main_renderer* renderer = renderer_holder_rc->renderer;

		{
			CPU_PROFILE_BLOCK("Sort render passes");
			ZoneScopedN("RenderSystem::submitRendererParams");

			opaqueRenderPass.sort();
			transparentRenderPass.sort();
			renderer_holder_rc->ldrRenderPass->sort();

			for (uint32 i = 0; i < sunShadowRenderPass.numCascades; ++i)
			{
				sunShadowRenderPass.cascades[i].sort();
			}

			for (uint32 i = 0; i < numSpotLightShadowPasses; ++i)
			{
				spotShadowRenderPasses[i].sort();
			}

			for (uint32 i = 0; i < numPointLightShadowPasses; ++i)
			{
				pointShadowRenderPasses[i].sort();
			}
		}

		renderer->submitRenderPass(&opaqueRenderPass);
		renderer->submitRenderPass(&transparentRenderPass);
		renderer->submitRenderPass(renderer_holder_rc->ldrRenderPass);
		renderer->submitComputePass(&computePass);

		renderer->submitShadowRenderPass(&sunShadowRenderPass);

		for (uint32 i = 0; i < numSpotLightShadowPasses; ++i)
		{
			renderer->submitShadowRenderPass(&spotShadowRenderPasses[i]);
		}

		for (uint32 i = 0; i < numPointLightShadowPasses; ++i)
		{
			renderer->submitShadowRenderPass(&pointShadowRenderPasses[i]);
		}
	}

}