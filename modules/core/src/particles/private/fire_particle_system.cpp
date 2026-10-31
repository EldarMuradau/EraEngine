// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "particles/fire_particle_system.h"

#include "dx/dx_pipeline.h"
#include "dx/dx_profiling.h"

#include "rendering/render_resources.h"
#include "rendering/render_utils.h"

namespace era_engine
{
    DXPipeline FireParticleSystem::emit_pipeline;
    DXPipeline FireParticleSystem::simulate_pipeline;
    DXPipeline FireParticleSystem::render_pipeline;

    void FireParticleSystem::initialize_pipeline()
    {
#define name "fire_particle_system"

        emit_pipeline = create_reloadable_pipeline(EMIT_PIPELINE_NAME(name));
        simulate_pipeline = create_reloadable_pipeline(SIMULATE_PIPELINE_NAME(name));

        auto desc = CREATE_GRAPHICS_PIPELINE
                        .input_layout(input_layout_position)
                        .render_targets(transparent_light_pass_formats, arraysize(transparent_light_pass_formats), depth_stencil_format)
                        //.additiveBlending(0)
                        .alpha_blending(0)
                        .depth_settings(true, false);

        render_pipeline = create_reloadable_pipeline(desc, {VERTEX_SHADER_NAME(name), PIXEL_SHADER_NAME(name)});

#undef name
    }

    FireParticleSystem::FireParticleSystem(uint32 max_num_particles, float emit_rate, const std::string& texture_filename, uint32 cols, uint32 rows, vec3 pos)
    {
        initialize(max_num_particles, emit_rate, texture_filename, cols, rows, pos);
    }

    void FireParticleSystem::initialize(uint32 max_num_particles, float emit_rate, const std::string& texture_filename, uint32 cols, uint32 rows, vec3 pos)
    {
        this->emit_rate = emit_rate;
        auto tex = load_texture_from_file_async(texture_filename);

        position = pos;

        if (!tex)
        {
            tex = RenderResources::white_texture;
        }

        ParticleSystem::initialize_as_billboard(sizeof(FireParticleData), max_num_particles, SORT_MODE_BACK_TO_FRONT);

        atlas.texture = tex;
        atlas.cols = cols;
        atlas.rows = rows;

        atlas_cb.initialize(rows, cols);

        settings.size_over_lifetime.values[0] = 0.25f;
        settings.size_over_lifetime.values[1] = 0.7f;

        settings.intensity_over_lifetime.ts[0] = 0.f;
        settings.intensity_over_lifetime.ts[1] = 0.1f;
        settings.intensity_over_lifetime.ts[2] = 0.25f;
        settings.intensity_over_lifetime.ts[3] = 1.f;
        settings.intensity_over_lifetime.values[0] = 0.f;
        settings.intensity_over_lifetime.values[1] = 0.8f;
        settings.intensity_over_lifetime.values[2] = 0.9f;
        settings.intensity_over_lifetime.values[3] = 1.f;
    }

    void FireParticleSystem::update(struct DXCommandList* cl, const CommonParticleSimulationData& common, float dt)
    {
        struct Setter : public ParticleParameterSetter
        {
            FireSimulationCB cb;

            virtual void set_root_parameters(DXCommandList* cl) override
            {
                cl->set_compute_32bit_constants(FIRE_PARTICLE_SYSTEM_COMPUTE_RS_CBV, cb);
            }
        };

        Setter s;
        s.cb.camera_position = common.camera_position;
        s.cb.emit_position = position;
        s.cb.frame_index = (uint32)dx_context.get_frame_id();

        update_internal(cl, emit_rate * dt, dt, emit_pipeline, simulate_pipeline, &s);
    }

    void FireParticleSystem::render(TransparentRenderPass* render_pass)
    {
        FireRenderingCB cb;
        cb.atlas = atlas_cb;
        cb.settings = settings;

        FireMaterial material;
        material.atlas = atlas;
        material.cbv = dx_context.upload_dynamic_constant_buffer(cb);

        render_pass->render_particles<FirePipeline>(billboard_mesh.vertex_buffer, billboard_mesh.index_buffer,
            get_draw_info(render_pipeline),
            material);
    }

    PIPELINE_SETUP_IMPL(FireParticleSystem::FirePipeline)
    {
        cl->set_pipeline_state(*render_pipeline.pipeline);
        cl->set_graphics_root_signature(*render_pipeline.root_signature);
        cl->set_graphics_dynamic_constant_buffer(FIRE_PARTICLE_SYSTEM_RENDERING_RS_CAMERA, common.camera_cbv);
        cl->set_primitive_topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    }

    PARTICLE_PIPELINE_RENDER_IMPL(FireParticleSystem::FirePipeline, FireParticleSystem::FireMaterial)
    {
        cl->set_graphics_dynamic_constant_buffer(FIRE_PARTICLE_SYSTEM_RENDERING_RS_CBV, rc.data.cbv);
        cl->set_descriptor_heap_srv(FIRE_PARTICLE_SYSTEM_RENDERING_RS_TEXTURE, 0, rc.data.atlas.texture);

        ParticleRenderPipeline::render(cl, view_proj, rc);
    }
}