// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "particles/smoke_particle_system.h"

#include "dx/dx_pipeline.h"

#include "rendering/render_resources.h"
#include "rendering/render_utils.h"

namespace era_engine
{
    DXPipeline SmokeParticleSystem::emit_pipeline;
    DXPipeline SmokeParticleSystem::simulate_pipeline;
    DXPipeline SmokeParticleSystem::render_pipeline;

    void SmokeParticleSystem::initialize_pipeline()
    {
#define name "smoke_particle_system"

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

    SmokeParticleSystem::SmokeParticleSystem(uint32 max_num_particles, float emit_rate, const std::string& texture_filename, uint32 cols, uint32 rows, vec3 pos)
    {
        initialize(max_num_particles, emit_rate, texture_filename, cols, rows, pos);
    }

    void SmokeParticleSystem::initialize(uint32 max_num_particles, float emit_rate, const std::string& texture_filename, uint32 cols, uint32 rows, vec3 pos)
    {
        this->emit_rate = emit_rate;
        auto tex = load_texture_from_file_async(texture_filename);
        position = pos;
        if (!tex)
        {
            tex = RenderResources::white_texture;
        }

        ParticleSystem::initialize_as_billboard(sizeof(SmokeParticleData), max_num_particles, SORT_MODE_BACK_TO_FRONT);

        atlas.texture = tex;
        atlas.cols = cols;
        atlas.rows = rows;

        atlas_cb.initialize(rows, cols);
    }

    void SmokeParticleSystem::update(struct DXCommandList* cl, const CommonParticleSimulationData& common, float dt)
    {
        struct Setter : public ParticleParameterSetter
        {
            DXDynamicConstantBuffer cb;

            virtual void set_root_parameters(DXCommandList* cl) override
            {
                cl->set_compute_dynamic_constant_buffer(SMOKE_PARTICLE_SYSTEM_COMPUTE_RS_CBV, cb);
            }
        };

        SmokeSimulationCB cb;
        cb.camera_position = common.camera_position;
        cb.emit_position = position;
        cb.frame_index = (uint32)dx_context.get_frame_id();
        cb.life_scale_from_distance = settings.life_scale_from_distance;

        Setter s;
        s.cb = dx_context.upload_dynamic_constant_buffer(cb);

        update_internal(cl, emit_rate * dt, dt, emit_pipeline, simulate_pipeline, &s);
    }

    void SmokeParticleSystem::render(TransparentRenderPass* render_pass)
    {
        SmokeRenderingCB cb;
        cb.atlas = atlas_cb;
        cb.intensity_over_lifetime = settings.intensity_over_lifetime;
        cb.atlas_progression_over_lifetime = settings.atlas_progression_over_lifetime;

        SmokeMaterial material;
        material.atlas = atlas;
        material.cbv = dx_context.upload_dynamic_constant_buffer(cb);

        render_pass->render_particles<SmokePipeline>(billboard_mesh.vertex_buffer, billboard_mesh.index_buffer,
            get_draw_info(render_pipeline),
            material);
    }

    PIPELINE_SETUP_IMPL(SmokeParticleSystem::SmokePipeline)
    {
        cl->set_pipeline_state(*render_pipeline.pipeline);
        cl->set_graphics_root_signature(*render_pipeline.root_signature);
        cl->set_graphics_dynamic_constant_buffer(SMOKE_PARTICLE_SYSTEM_RENDERING_RS_CAMERA, common.camera_cbv);
        cl->set_primitive_topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    }

    PARTICLE_PIPELINE_RENDER_IMPL(SmokeParticleSystem::SmokePipeline, SmokeParticleSystem::SmokeMaterial)
    {
        cl->set_graphics_dynamic_constant_buffer(SMOKE_PARTICLE_SYSTEM_RENDERING_RS_CBV, rc.data.cbv);
        cl->set_descriptor_heap_srv(SMOKE_PARTICLE_SYSTEM_RENDERING_RS_TEXTURE, 0, rc.data.atlas.texture);

        ParticleRenderPipeline::render(cl, view_proj, rc);
    }
}