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
    void load_common_shaders();

    static vec2 halton_sequence[128];

    void initialize_render_utils()
    {
        initialize_texture_preprocessing();
        initialize_depth_prepass_pipelines();
        initialize_outline_pipelines();
        animation::initialize_skinning();
        initialize_shadow_pipelines();
        initialize_terrain_pipelines();
        initialize_light_probe_pipelines();
        initialize_rt_reflections_pipelines();
        initialize_procedural_placement_pipelines();
        initialize_grass_pipelines();
        initialize_water_pipelines();
        initialize_tree_pipelines();
        load_common_shaders();

        DebugSimplePipeline::initialize();
        DebugUnlitPipeline::initialize();
        DebugUnlitLinePipeline::initialize();
        PBRPipeline::initialize();
        ParticleSystem::initialize_pipeline();
        initialize_bitonic_sort();
        load_all_particle_system_pipelines();

        create_all_pending_reloadable_pipelines();
        RenderResources::initialize_global_resources();

        for (uint32 i = 0; i < arraysize(halton_sequence); ++i)
        {
            halton_sequence[i] = halton23(i) * 2.f - vec2(1.f);
        }
    }

    void end_frame_common()
    {
        check_for_changed_pipelines();
    }

    void build_camera_constant_buffer(const RenderCamera& camera, float camera_jitter_strength, CameraCB& out_cb)
    {
        if (camera_jitter_strength > 0.f)
        {
            vec2 jitter = halton_sequence[dx_context.get_frame_id() % arraysize(halton_sequence)] / vec2((float)camera.width, (float)camera.height) * camera_jitter_strength;
            build_camera_constant_buffer(camera.get_jittered_version(jitter), jitter, out_cb);
        }
        else
        {
            build_camera_constant_buffer(camera, vec2(0.f, 0.f), out_cb);
        }
    }

    void build_camera_constant_buffer(const RenderCamera& camera, vec2 jitter, CameraCB& out_cb)
    {
        out_cb.prev_frame_view = out_cb.view;
        out_cb.prev_frame_view_proj = out_cb.view_proj;
        out_cb.view_proj = camera.view_proj;
        out_cb.view = camera.view;
        out_cb.proj = camera.proj;
        out_cb.inv_view_proj = camera.inv_view_proj;
        out_cb.inv_view = camera.inv_view;
        out_cb.inv_proj = camera.inv_proj;
        out_cb.position = vec4(camera.position, 1.f);
        out_cb.forward = vec4(camera.rotation * vec3(0.f, 0.f, -1.f), 0.f);
        out_cb.right = vec4(camera.rotation * vec3(1.f, 0.f, 0.f), 0.f);
        out_cb.up = vec4(camera.rotation * vec3(0.f, 1.f, 0.f), 0.f);
        out_cb.projection_params = vec4(camera.near_plane, camera.far_plane, camera.far_plane / camera.near_plane, 1.f - camera.far_plane / camera.near_plane);
        out_cb.view_space_top_left_frustum_vector = vec4(camera.restore_view_space_position(vec2(0.f, 0.f), 0.f) / camera.near_plane, 0.f);
        out_cb.extent_at_distance_one = abs(camera.restore_view_space_position(vec2(1.f, 1.f), 0.f).xy / camera.near_plane - out_cb.view_space_top_left_frustum_vector.xy);
        out_cb.screen_dims = vec2((float)camera.width, (float)camera.height);
        out_cb.inv_screen_dims = vec2(1.f / camera.width, 1.f / camera.height);
        out_cb.prev_frame_jitter = out_cb.jitter;
        out_cb.jitter = jitter;
    }

    RaytracingObjectType define_blas_from_mesh(const ref<MultiMesh>& mesh)
    {
        if (dx_context.feature_support.raytracing())
        {
            RaytracingBLASBuilder blas_builder;
            std::vector<ref<PBRMaterial>> raytracing_materials;

            for (auto& sm : mesh->submeshes)
            {
                blas_builder.push(mesh->mesh.vertex_buffer, mesh->mesh.index_buffer, sm.info);
                raytracing_materials.push_back(sm.material ? sm.material : get_default_pbr_material());
            }

            ref<RaytracingBLAS> blas = blas_builder.finish();
            RaytracingObjectType type = PBRRaytracer::define_object_type(blas, raytracing_materials);
            return type;
        }
        else
        {
            return {};
        }
    }
}