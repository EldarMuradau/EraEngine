#pragma once

#include "ecs/system.h"

#include "core/memory.h"

#include "geometry/mesh.h"

#include "particles/fire_particle_system.h"
#include "particles/smoke_particle_system.h"
#include "particles/boid_particle_system.h"
#include "particles/debris_particle_system.h"

#include "rendering/raytracing.h"
#include "rendering/main_renderer.h"

namespace era_engine
{
	class RendererHolderRootComponent;

	class RenderSystem final : public System
	{
	public:
		RenderSystem(World* _world);
		~RenderSystem();

		void init() override;
		void begin_frame(float dt);
		void before_render(float dt);
		void update(float dt) override;
		void after_render(float dt);

		ERA_VIRTUAL_REFLECT(System)

	private:
		void load_custom_shaders();

		void resetRenderPasses();
		void submitRendererParams(uint32 numSpotLightShadowPasses, uint32 numPointLightShadowPasses);

	private:
		RendererHolderRootComponent* renderer_holder_rc = nullptr;

		Allocator stackArena;

		raytracing_tlas raytracingTLAS;

		ref<dx_buffer> pointLightBuffer[NUM_BUFFERED_FRAMES];
		ref<dx_buffer> spotLightBuffer[NUM_BUFFERED_FRAMES];
		ref<dx_buffer> decalBuffer[NUM_BUFFERED_FRAMES];

		ref<dx_buffer> spotLightShadowInfoBuffer[NUM_BUFFERED_FRAMES];
		ref<dx_buffer> pointLightShadowInfoBuffer[NUM_BUFFERED_FRAMES];

		std::vector<pbr_decal_cb> decals;

		ref<dx_texture> decalTexture;

		fire_particle_system fireParticleSystem;
		smoke_particle_system smokeParticleSystem;
		boid_particle_system boidParticleSystem;
		debris_particle_system debrisParticleSystem;

		opaque_render_pass opaqueRenderPass;
		transparent_render_pass transparentRenderPass;
		sun_shadow_render_pass sunShadowRenderPass;
		spot_shadow_render_pass spotShadowRenderPasses[16];
		point_shadow_render_pass pointShadowRenderPasses[16];
		compute_pass computePass;
	};
}