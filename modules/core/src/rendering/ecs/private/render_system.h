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
        ~RenderSystem() override;

        void init() override;
        void begin_frame(float dt);
        void before_render(float dt);
        void update(float dt) override;
        void after_render(float dt);

        ERA_VIRTUAL_REFLECT(System)

    private:
        void load_custom_shaders();
        void draw_imgui_viewport();
        void instantiate_dropped_prefab(const char* path);

        void reset_render_passes();
        void submit_renderer_params(uint32 num_spot_light_shadow_passes, uint32 num_point_light_shadow_passes);

        void build_raytracing_scene();
        const RaytracingObjectType* get_raytracing_object_type(const ref<MultiMesh>& mesh);

    private:
        RendererHolderRootComponent* renderer_holder_rc = nullptr;

        Allocator stack_arena;

        struct RaytracingMeshEntry
        {
            weakref<MultiMesh> mesh;
            RaytracingObjectType type;
        };

        // BLAS is shared by all entities with the same mesh.
        std::unordered_map<const MultiMesh*, RaytracingMeshEntry> raytracing_mesh_types;

        ref<DXBuffer> point_light_buffer[NUM_BUFFERED_FRAMES];
        ref<DXBuffer> spot_light_buffer[NUM_BUFFERED_FRAMES];
        ref<DXBuffer> decal_buffer[NUM_BUFFERED_FRAMES];

        ref<DXBuffer> spot_light_shadow_info_buffer[NUM_BUFFERED_FRAMES];
        ref<DXBuffer> point_light_shadow_info_buffer[NUM_BUFFERED_FRAMES];

        std::vector<PBRDecalCB> decals;

        ref<DXTexture> decal_texture;

        FireParticleSystem fire_particle_system;
        SmokeParticleSystem smoke_particle_system;
        BoidParticleSystem boid_particle_system;
        DebrisParticleSystem debris_particle_system;

        struct FramePasses
        {
            OpaqueRenderPass opaque_render_pass;
            TransparentRenderPass transparent_render_pass;
            LDRRenderPass ldr_render_pass;
            SunShadowRenderPass sun_shadow_render_pass;
            SpotShadowRenderPass spot_shadow_render_passes[16];
            PointShadowRenderPass point_shadow_render_passes[16];
            ComputePass compute_pass;
            RaytracingTLAS raytracing_tlas;
        };

        FramePasses& frame_passes();

        FramePasses passes[NUM_BUFFERED_FRAMES];
    };
}