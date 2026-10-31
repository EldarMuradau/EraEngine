// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "particles/debris_particle_system.h"

#include "core/log.h"

#include "dx/dx_pipeline.h"
#include "dx/dx_profiling.h"

#include "rendering/render_resources.h"
#include "rendering/render_utils.h"

namespace era_engine
{
    DXPipeline DebrisParticleSystem::emit_pipeline;
    DXPipeline DebrisParticleSystem::simulate_pipeline;
    DXPipeline DebrisParticleSystem::render_pipeline;

    void DebrisParticleSystem::initialize_pipeline()
    {
#define name "debris_particle_system"

        emit_pipeline = create_reloadable_pipeline(EMIT_PIPELINE_NAME(name));
        simulate_pipeline = create_reloadable_pipeline(SIMULATE_PIPELINE_NAME(name));

        auto desc = CREATE_GRAPHICS_PIPELINE
                        .input_layout(input_layout_position)
                        .render_targets(transparent_light_pass_formats, arraysize(transparent_light_pass_formats), depth_stencil_format)
                        .alpha_blending(0)
                        .depth_settings(true, false);

        render_pipeline = create_reloadable_pipeline(desc, {VERTEX_SHADER_NAME(name), PIXEL_SHADER_NAME(name)});

#undef name
    }

    DebrisParticleSystem::DebrisParticleSystem(uint32 max_num_particles)
    {
        initialize(max_num_particles);
    }

    void DebrisParticleSystem::initialize(uint32 max_num_particles)
    {
        ParticleSystem::initialize_as_billboard(sizeof(DebrisParticleData), max_num_particles, SORT_MODE_BACK_TO_FRONT);
    }

    void DebrisParticleSystem::burst(vec3 position)
    {
        bursts.push_back({position});
    }

    void DebrisParticleSystem::update(struct DXCommandList* cl, const CommonParticleSimulationData& common, float dt)
    {
        struct Setter : public ParticleParameterSetter
        {
            DXDynamicConstantBuffer cbv;
            ref<DXTexture> depth_buffer;
            ref<DXTexture> normals;

            virtual void set_root_parameters(DXCommandList* cl) override
            {
                cl->set_compute_dynamic_constant_buffer(DEBRIS_PARTICLE_SYSTEM_COMPUTE_RS_CBV, cbv);
                cl->set_descriptor_heap_srv(DEBRIS_PARTICLE_SYSTEM_COMPUTE_RS_TEXTURES, 0, depth_buffer);
                cl->set_descriptor_heap_srv(DEBRIS_PARTICLE_SYSTEM_COMPUTE_RS_TEXTURES, 1, normals);
            }
        };

        const float linear_damping = 0.4f;

        DebrisSimulationCB cb;
        cb.camera_vp = common.prev_frame_camera_view_proj;
        cb.camera_projection_params = common.camera_projection_params;
        cb.camera_position = common.camera_position;
        cb.frame_index = (uint32)dx_context.get_frame_id();
        cb.drag = 1.f / (1.f + dt * linear_damping);

        uint32 num_bursts = (uint32)min(arraysize(cb.emit_positions), bursts.size());
        uint32 num_new_particles = num_bursts * 256;
        for (uint32 i = 0; i < num_bursts; ++i)
        {
            cb.emit_positions[i] = vec4(bursts[i].position, 1.f);
        }

        Setter s;
        s.cbv = dx_context.upload_dynamic_constant_buffer(cb);
        s.depth_buffer = common.prev_frame_depth_buffer;
        s.normals = common.prev_frame_normals;

        update_internal(cl, (float)num_new_particles, dt, emit_pipeline, simulate_pipeline, &s);

        bursts.clear();
    }

    void DebrisParticleSystem::render(TransparentRenderPass* render_pass)
    {
        DebrisMaterial material;

        render_pass->render_particles<DebrisPipeline>(billboard_mesh.vertex_buffer, billboard_mesh.index_buffer,
            get_draw_info(render_pipeline),
            material);
    }

    PIPELINE_SETUP_IMPL(DebrisParticleSystem::DebrisPipeline)
    {
        cl->set_pipeline_state(*render_pipeline.pipeline);
        cl->set_graphics_root_signature(*render_pipeline.root_signature);
        cl->set_primitive_topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        cl->set_graphics_dynamic_constant_buffer(DEBRIS_PARTICLE_SYSTEM_RENDERING_RS_CAMERA, common.camera_cbv);
    }

    PARTICLE_PIPELINE_RENDER_IMPL(DebrisParticleSystem::DebrisPipeline, DebrisParticleSystem::DebrisMaterial)
    {
        ParticleRenderPipeline::render(cl, view_proj, rc);
    }
}