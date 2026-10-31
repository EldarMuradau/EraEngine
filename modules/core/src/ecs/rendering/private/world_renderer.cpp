#include "ecs/rendering/world_renderer.h"
#include "ecs/rendering/mesh_component.h"
#include "ecs/rendering/cloth_render_component.h"
#include "ecs/base_components/base_components.h"

#include "core/cpu_profiling.h"
#include "core/string.h"

#include "rendering/pbr.h"
#include "rendering/depth_prepass.h"
#include "rendering/outline.h"
#include "rendering/shadow_map.h"

#include "animation/animation.h"

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

    static bool should_render(BoundingSphere s, BoundingBox aabb, const trs& transform)
    {
        s.center = conjugate(transform.rotation) * (s.center - transform.position) + transform.position;
        aabb.min_corner *= transform.scale;
        aabb.max_corner *= transform.scale;
        return aabb.contains(s.center) || sphere_vs_aabb(s, aabb);
    }

    static bool should_render(const CameraFrustumPlanes& frustum, const MeshComponent& mesh, const TransformComponent& transform)
    {
        return mesh.mesh && !mesh.is_hidden && mesh.mesh->has_visible_submeshes() && (mesh.mesh->load_state.load() == AssetLoadState::LOADED) && ((mesh.mesh->aabb.max_corner.x == mesh.mesh->aabb.min_corner.x) || !frustum.cull_model_space_aabb(mesh.mesh->aabb, transform.get_world_transform()));
    }

    static bool should_render(const BoundingSphere& frustum, const MeshComponent& mesh, const TransformComponent& transform)
    {
        return mesh.mesh && !mesh.is_hidden && mesh.mesh->has_visible_submeshes() && (mesh.mesh->load_state.load() == AssetLoadState::LOADED) && ((mesh.mesh->aabb.max_corner.x == mesh.mesh->aabb.min_corner.x) || should_render(frustum, mesh.mesh->aabb, transform.get_world_transform()));
    }

    static bool should_render(const LightFrustum& frustum, const MeshComponent& mesh, const TransformComponent& transform)
    {
        return (frustum.type == LIGHT_FRUSTUM_STANDARD) ? should_render(frustum.frustum, mesh, transform) : should_render(frustum.sphere, mesh, transform);
    }

    template <typename group_t>
    std::unordered_map<MultiMesh*, OffsetCount> get_offsets_per_mesh(group_t group)
    {
        uint32 group_size = (uint32)group.size();

        std::unordered_map<MultiMesh*, OffsetCount> oc_per_mesh;

        uint32 index = 0;
        for (Entity::Handle entity_handle : group)
        {
            MeshComponent& mesh = group.get<MeshComponent>(entity_handle);
            ++oc_per_mesh[mesh.mesh.get()].count;
        }

        uint32 offset = 0;
        for (auto& [mesh, oc] : oc_per_mesh)
        {
            oc.offset = offset;
            offset += oc.count;
            oc.count = 0;
        }

        oc_per_mesh.erase(nullptr);

        return oc_per_mesh;
    }

    static void add_to_render_pass(PbrMaterialShader shader, const PBRRenderData& data, const DepthPrepassData& depth_prepass_data,
        OpaqueRenderPass* opaque_render_pass, TransparentRenderPass* transparent_render_pass)
    {
        switch (shader)
        {
            case PBR_MATERIAL_SHADER_DEFAULT:
            case PBR_MATERIAL_SHADER_DOUBLE_SIDED:
            {
                if (shader == PBR_MATERIAL_SHADER_DEFAULT)
                {
                    opaque_render_pass->render_object<PBRPipeline::Opaque>(data);
                    opaque_render_pass->render_depth_only<DepthPrepassPipeline::SingleSided>(depth_prepass_data);
                }
                else
                {
                    opaque_render_pass->render_object<PBRPipeline::OpaqueDoubleSided>(data);
                    opaque_render_pass->render_depth_only<DepthPrepassPipeline::DoubleSided>(depth_prepass_data);
                }
            }
            break;
            case PBR_MATERIAL_SHADER_ALPHA_CUTOUT:
            {
                opaque_render_pass->render_object<PBRPipeline::OpaqueDoubleSided>(data);
                opaque_render_pass->render_depth_only<DepthPrepassPipeline::AlphaCutout>(depth_prepass_data);
            }
            break;
            case PBR_MATERIAL_SHADER_TRANSPARENT:
            {
                transparent_render_pass->render_object<PBRPipeline::Transparent>(data);
                transparent_render_pass->render_motion_vectors<DepthPrepassPipeline::TransparentMotionVectors>(depth_prepass_data);
            }
            break;
        }
    }

    static void add_to_static_render_pass(PbrMaterialShader shader, const ShadowRenderData& shadow_data, ShadowRenderPassBase* shadow_render_pass, bool is_point_light)
    {
        switch (shader)
        {
            case PBR_MATERIAL_SHADER_DEFAULT:
            {
                if (is_point_light)
                {
                    shadow_render_pass->render_static_object<PointShadowPipeline::SingleSided>(shadow_data);
                }
                else
                {
                    shadow_render_pass->render_static_object<ShadowPipeline::SingleSided>(shadow_data);
                }
            }
            break;
            case PBR_MATERIAL_SHADER_DOUBLE_SIDED:
            case PBR_MATERIAL_SHADER_ALPHA_CUTOUT:
            {
                if (is_point_light)
                {
                    shadow_render_pass->render_static_object<PointShadowPipeline::DoubleSided>(shadow_data);
                }
                else
                {
                    shadow_render_pass->render_static_object<ShadowPipeline::DoubleSided>(shadow_data);
                }
            }
            break;
            case PBR_MATERIAL_SHADER_TRANSPARENT:
            {
                // Do nothing
            }
            break;
        }
    }

    static void add_to_dynamic_render_pass(PbrMaterialShader shader, const ShadowRenderData& shadow_data, ShadowRenderPassBase* shadow_render_pass, bool is_point_light)
    {
        switch (shader)
        {
            case PBR_MATERIAL_SHADER_DEFAULT:
            {
                if (is_point_light)
                {
                    shadow_render_pass->render_dynamic_object<PointShadowPipeline::SingleSided>(shadow_data);
                }
                else
                {
                    shadow_render_pass->render_dynamic_object<ShadowPipeline::SingleSided>(shadow_data);
                }
            }
            break;
            case PBR_MATERIAL_SHADER_DOUBLE_SIDED:
            case PBR_MATERIAL_SHADER_ALPHA_CUTOUT:
            {
                if (is_point_light)
                {
                    shadow_render_pass->render_dynamic_object<PointShadowPipeline::DoubleSided>(shadow_data);
                }
                else
                {
                    shadow_render_pass->render_dynamic_object<ShadowPipeline::DoubleSided>(shadow_data);
                }
            }
            break;
            case PBR_MATERIAL_SHADER_TRANSPARENT:
            {
                // Do nothing
            }
            break;
        }
    }

    template <typename group_t>
    static void render_static_objects_to_main_camera(group_t group, std::unordered_map<MultiMesh*, OffsetCount> oc_per_mesh,
        const CameraFrustumPlanes& frustum, Allocator& arena, Entity::Handle selected_object_id,
        OpaqueRenderPass* opaque_render_pass, TransparentRenderPass* transparent_render_pass, LDRRenderPass* ldr_render_pass)
    {
        uint32 group_size = (uint32)group.size();

        DXAllocation transform_allocation = dx_context.allocate_dynamic_buffer(group_size * sizeof(mat4), 4);
        mat4* transforms = (mat4*)transform_allocation.cpu_ptr;

        DXAllocation object_id_allocation = dx_context.allocate_dynamic_buffer(group_size * sizeof(uint32), 4);
        uint32* object_ids = (uint32*)object_id_allocation.cpu_ptr;

        for (auto [entity_handle, transform, mesh] : group.each())
        {
            if (!should_render(frustum, mesh, transform))
                continue;

            if (transform.type == TransformComponent::DYNAMIC)
                continue;

            std::lock_guard<std::mutex> lock{mesh.mesh->mutex};

            const DXMesh& dx_mesh = mesh.mesh->mesh;

            OffsetCount& oc = oc_per_mesh.at(mesh.mesh.get());

            uint32 index = oc.offset + oc.count;
            const trs& world_transform = transform.get_world_transform();

            transforms[index] = trs_to_mat4(world_transform);
            object_ids[index] = (uint32)entity_handle;

            ++oc.count;

            if (entity_handle == selected_object_id)
            {
                for (Submesh& sm : mesh.mesh->submeshes)
                {
                    if (sm.is_visible)
                    {
                        render_outline(ldr_render_pass, transforms[index], dx_mesh.vertex_buffer, dx_mesh.index_buffer, sm.info);
                    }
                }
            }
        }

        D3D12_GPU_VIRTUAL_ADDRESS transforms_address = transform_allocation.gpu_ptr;
        D3D12_GPU_VIRTUAL_ADDRESS object_id_address = object_id_allocation.gpu_ptr;

        uint32 num_draw_calls = 0;

        for (auto& [mesh, oc] : oc_per_mesh)
        {
            if (oc.count == 0)
                continue;

            std::lock_guard<std::mutex> lock{mesh->mutex};

            D3D12_GPU_VIRTUAL_ADDRESS base_m = transforms_address + (oc.offset * sizeof(mat4));
            D3D12_GPU_VIRTUAL_ADDRESS base_object_id = object_id_address + (oc.offset * sizeof(uint32));

            const DXMesh& dx_mesh = mesh->mesh;

            if (!dx_mesh.index_buffer || !dx_mesh.vertex_buffer.positions)
                continue;

            PBRRenderData data;
            data.transform_ptr = base_m;
            data.vertex_buffer = dx_mesh.vertex_buffer;
            data.index_buffer = dx_mesh.index_buffer;
            data.num_instances = oc.count;

            DepthPrepassData depth_prepass_data;
            depth_prepass_data.transform_ptr = base_m;
            depth_prepass_data.prev_frame_transform_ptr = base_m;
            depth_prepass_data.object_id_ptr = base_object_id;
            depth_prepass_data.vertex_buffer = dx_mesh.vertex_buffer;
            depth_prepass_data.prev_frame_vertex_buffer = dx_mesh.vertex_buffer.positions;
            depth_prepass_data.index_buffer = dx_mesh.index_buffer;
            depth_prepass_data.num_instances = oc.count;

            for (auto& sm : mesh->submeshes)
            {
                if (!sm.is_visible)
                {
                    continue;
                }

                data.submesh = sm.info;
                data.material = sm.material;

                depth_prepass_data.submesh = data.submesh;
                depth_prepass_data.alpha_cutout_texture_srv = (sm.material && sm.material->albedo) ? sm.material->albedo->default_srv : DXCPUDescriptorHandle{};

                add_to_render_pass(sm.material->shader, data, depth_prepass_data, opaque_render_pass, transparent_render_pass);

                ++num_draw_calls;
            }
        }

        CPU_PROFILE_STAT("Static draw calls", num_draw_calls);
    }

    template <typename group_t>
    static void render_static_objects_to_shadow_map(group_t group, std::unordered_map<MultiMesh*, OffsetCount> oc_per_mesh,
        const LightFrustum& frustum, Allocator& arena, ShadowRenderPassBase* shadow_render_pass)
    {
        uint32 group_size = (uint32)group.size();

        DXAllocation transform_allocation = dx_context.allocate_dynamic_buffer(group_size * sizeof(mat4), 4);
        mat4* transforms = (mat4*)transform_allocation.cpu_ptr;

        for (auto [entity_handle, transform, mesh] : group.each())
        {
            if (!should_render(frustum, mesh, transform))
                continue;

            if (transform.type == TransformComponent::DYNAMIC)
                continue;

            std::lock_guard<std::mutex> lock{mesh.mesh->mutex};

            const DXMesh& dx_mesh = mesh.mesh->mesh;

            OffsetCount& oc = oc_per_mesh.at(mesh.mesh.get());

            uint32 index = oc.offset + oc.count;
            const trs& world_transform = transform.get_world_transform();

            transforms[index] = trs_to_mat4(world_transform);

            ++oc.count;
        }

        D3D12_GPU_VIRTUAL_ADDRESS transforms_address = transform_allocation.gpu_ptr;

        for (auto& [mesh, oc] : oc_per_mesh)
        {
            if (oc.count == 0)
                continue;

            std::lock_guard<std::mutex> lock{mesh->mutex};

            D3D12_GPU_VIRTUAL_ADDRESS base_m = transforms_address + (oc.offset * sizeof(mat4));

            const DXMesh& dx_mesh = mesh->mesh;

            if (!dx_mesh.index_buffer || !dx_mesh.vertex_buffer.positions)
                continue;

            ShadowRenderData data;
            data.transform_ptr = base_m;
            data.vertex_buffer = dx_mesh.vertex_buffer.positions;
            data.index_buffer = dx_mesh.index_buffer;
            data.num_instances = oc.count;

            for (auto& sm : mesh->submeshes)
            {
                if (!sm.is_visible)
                {
                    continue;
                }
                data.submesh = sm.info;
                add_to_static_render_pass(sm.material->shader, data, shadow_render_pass, frustum.type == LIGHT_FRUSTUM_SPHERE);
            }
        }
    }

    static void render_static_objects(World* world, const CameraFrustumPlanes& frustum, Allocator& arena, Entity::Handle selected_object_id,
        OpaqueRenderPass* opaque_render_pass, TransparentRenderPass* transparent_render_pass, LDRRenderPass* ldr_render_pass, ShadowPasses& shadow)
    {
        CPU_PROFILE_BLOCK("Static objects");

        using SpecializedComponents = ComponentsGroup<
            animation::AnimationComponent,
            TreeComponent>;

        auto group = world->group(
            components_group<TransformComponent, MeshComponent>,
            SpecializedComponents{});

        std::unordered_map<MultiMesh*, OffsetCount> oc_per_mesh = get_offsets_per_mesh(group);

        render_static_objects_to_main_camera(group, oc_per_mesh, frustum, arena, selected_object_id, opaque_render_pass, transparent_render_pass, ldr_render_pass);

        for (uint32 i = 0; i < shadow.num_shadow_render_passes; ++i)
        {
            auto& pass = shadow.shadow_render_passes[i];
            render_static_objects_to_shadow_map(group, oc_per_mesh, pass.frustum, arena, pass.pass);
        }
    }

    template <typename group_t>
    static void render_dynamic_objects_to_main_camera(group_t group, std::unordered_map<MultiMesh*, OffsetCount> oc_per_mesh,
        const CameraFrustumPlanes& frustum, Allocator& arena, Entity::Handle selected_object_id,
        OpaqueRenderPass* opaque_render_pass, TransparentRenderPass* transparent_render_pass, LDRRenderPass* ldr_render_pass)
    {
        uint32 group_size = (uint32)group.size();

        DXAllocation transform_allocation = dx_context.allocate_dynamic_buffer(group_size * sizeof(mat4) * 2, 4);
        mat4* transforms = (mat4*)transform_allocation.cpu_ptr;
        mat4* prev_frame_transforms = transforms + group_size;

        DXAllocation object_id_allocation = dx_context.allocate_dynamic_buffer(group_size * sizeof(uint32), 4);
        uint32* object_ids = (uint32*)object_id_allocation.cpu_ptr;

        for (auto [entity_handle, transform, mesh] : group.each())
        {
            if (!should_render(frustum, mesh, transform))
                continue;

            if (transform.type != TransformComponent::DYNAMIC)
                continue;

            const DXMesh& dx_mesh = mesh.mesh->mesh;

            OffsetCount& oc = oc_per_mesh.at(mesh.mesh.get());

            uint32 index = oc.offset + oc.count;
            const trs& world_transform = transform.get_world_transform();

            transforms[index] = trs_to_mat4(world_transform);
            prev_frame_transforms[index] = mesh.update_frame_transform(transforms[index], dx_context.get_frame_id());
            object_ids[index] = (uint32)entity_handle;

            ++oc.count;

            if (entity_handle == selected_object_id)
            {
                for (auto& sm : mesh.mesh->submeshes)
                {
                    if (!sm.is_visible)
                    {
                        continue;
                    }
                    render_outline(ldr_render_pass, transforms[index], dx_mesh.vertex_buffer, dx_mesh.index_buffer, sm.info);
                }
            }
        }

        D3D12_GPU_VIRTUAL_ADDRESS transforms_address = transform_allocation.gpu_ptr;
        D3D12_GPU_VIRTUAL_ADDRESS prev_frame_transforms_address = transform_allocation.gpu_ptr + (group_size * sizeof(mat4));
        D3D12_GPU_VIRTUAL_ADDRESS object_id_address = object_id_allocation.gpu_ptr;

        uint32 num_draw_calls = 0;

        for (auto& [mesh, oc] : oc_per_mesh)
        {
            if (oc.count == 0)
                continue;

            D3D12_GPU_VIRTUAL_ADDRESS base_m = transforms_address + (oc.offset * sizeof(mat4));
            D3D12_GPU_VIRTUAL_ADDRESS prev_base_m = prev_frame_transforms_address + (oc.offset * sizeof(mat4));
            D3D12_GPU_VIRTUAL_ADDRESS base_object_id = object_id_address + (oc.offset * sizeof(uint32));

            const DXMesh& dx_mesh = mesh->mesh;

            if (!dx_mesh.index_buffer || !dx_mesh.vertex_buffer.positions)
                continue;

            PBRRenderData data;
            data.transform_ptr = base_m;
            data.vertex_buffer = dx_mesh.vertex_buffer;
            data.index_buffer = dx_mesh.index_buffer;
            data.num_instances = oc.count;

            DepthPrepassData depth_prepass_data;
            depth_prepass_data.transform_ptr = base_m;
            depth_prepass_data.prev_frame_transform_ptr = prev_base_m;
            depth_prepass_data.object_id_ptr = base_object_id;
            depth_prepass_data.vertex_buffer = dx_mesh.vertex_buffer;
            depth_prepass_data.prev_frame_vertex_buffer = dx_mesh.vertex_buffer.positions;
            depth_prepass_data.index_buffer = dx_mesh.index_buffer;
            depth_prepass_data.num_instances = oc.count;

            for (auto& sm : mesh->submeshes)
            {
                if (!sm.is_visible)
                {
                    continue;
                }
                data.submesh = sm.info;
                data.material = sm.material;

                depth_prepass_data.submesh = data.submesh;
                depth_prepass_data.alpha_cutout_texture_srv = (sm.material && sm.material->albedo) ? sm.material->albedo->default_srv : DXCPUDescriptorHandle{};

                add_to_render_pass(sm.material->shader, data, depth_prepass_data, opaque_render_pass, transparent_render_pass);

                ++num_draw_calls;
            }
        }

        CPU_PROFILE_STAT("Dynamic draw calls", num_draw_calls);
    }

    template <typename group_t>
    static void render_dynamic_objects_to_shadow_map(group_t group, std::unordered_map<MultiMesh*, OffsetCount> oc_per_mesh,
        const LightFrustum& frustum, Allocator& arena, ShadowRenderPassBase* shadow_render_pass)
    {
        uint32 group_size = (uint32)group.size();

        DXAllocation transform_allocation = dx_context.allocate_dynamic_buffer(group_size * sizeof(mat4), 4);
        mat4* transforms = (mat4*)transform_allocation.cpu_ptr;

        for (auto [entity_handle, transform, mesh] : group.each())
        {
            if (!should_render(frustum, mesh, transform))
                continue;

            if (transform.type != TransformComponent::DYNAMIC)
                continue;

            std::lock_guard<std::mutex> lock{mesh.mesh->mutex};

            const DXMesh& dx_mesh = mesh.mesh->mesh;
            if (!oc_per_mesh.contains(mesh.mesh.get()))
                continue;
            OffsetCount& oc = oc_per_mesh.at(mesh.mesh.get());

            uint32 index = oc.offset + oc.count;
            const trs& world_transform = transform.get_world_transform();

            transforms[index] = trs_to_mat4(world_transform);

            ++oc.count;
        }

        D3D12_GPU_VIRTUAL_ADDRESS transforms_address = transform_allocation.gpu_ptr;

        for (auto& [mesh, oc] : oc_per_mesh)
        {
            if (oc.count == 0)
                continue;

            std::lock_guard<std::mutex> lock{mesh->mutex};

            D3D12_GPU_VIRTUAL_ADDRESS base_m = transforms_address + (oc.offset * sizeof(mat4));

            const DXMesh& dx_mesh = mesh->mesh;

            if (!dx_mesh.index_buffer || !dx_mesh.vertex_buffer.positions)
                continue;

            ShadowRenderData data;
            data.transform_ptr = base_m;
            data.vertex_buffer = dx_mesh.vertex_buffer.positions;
            data.index_buffer = dx_mesh.index_buffer;
            data.num_instances = oc.count;

            for (auto& sm : mesh->submeshes)
            {
                if (!sm.is_visible)
                {
                    continue;
                }
                data.submesh = sm.info;
                add_to_dynamic_render_pass(sm.material->shader, data, shadow_render_pass, frustum.type == LIGHT_FRUSTUM_SPHERE);
            }
        }
    }

    static void render_dynamic_objects(World* world, const CameraFrustumPlanes& frustum, Allocator& arena, Entity::Handle selected_object_id,
        OpaqueRenderPass* opaque_render_pass, TransparentRenderPass* transparent_render_pass, LDRRenderPass* ldr_render_pass, ShadowPasses& shadow)
    {
        CPU_PROFILE_BLOCK("Dynamic objects");

        auto group = world->group(
            components_group<TransformComponent, MeshComponent>,
            components_group<animation::AnimationComponent>);

        std::unordered_map<MultiMesh*, OffsetCount> oc_per_mesh = get_offsets_per_mesh(group);
        render_dynamic_objects_to_main_camera(group, oc_per_mesh, frustum, arena, selected_object_id, opaque_render_pass, transparent_render_pass, ldr_render_pass);

        for (uint32 i = 0; i < shadow.num_shadow_render_passes; ++i)
        {
            auto& pass = shadow.shadow_render_passes[i];
            render_dynamic_objects_to_shadow_map(group, oc_per_mesh, pass.frustum, arena, pass.pass);
        }
    }

    static void render_animated_objects(World* world, const CameraFrustumPlanes& frustum, Allocator& arena, Entity::Handle selected_object_id,
        OpaqueRenderPass* opaque_render_pass, TransparentRenderPass* transparent_render_pass, LDRRenderPass* ldr_render_pass, ShadowPasses& shadow)
    {
        CPU_PROFILE_BLOCK("Animated objects");

        auto group = world->group(
            components_group<TransformComponent, MeshComponent, animation::AnimationComponent>);

        uint32 group_size = (uint32)group.size();

        if (group_size == 0)
        {
            return;
        }

        DXAllocation transform_allocation = dx_context.allocate_dynamic_buffer(group_size * sizeof(mat4) * 2, 4);
        mat4* transforms = (mat4*)transform_allocation.cpu_ptr;
        mat4* prev_frame_transforms = transforms + group_size;

        DXAllocation object_id_allocation = dx_context.allocate_dynamic_buffer(group_size * sizeof(uint32), 4);
        uint32* object_ids = (uint32*)object_id_allocation.cpu_ptr;

        D3D12_GPU_VIRTUAL_ADDRESS transforms_address = transform_allocation.gpu_ptr;
        D3D12_GPU_VIRTUAL_ADDRESS prev_frame_transforms_address = transform_allocation.gpu_ptr + (group_size * sizeof(mat4));
        D3D12_GPU_VIRTUAL_ADDRESS object_id_address = object_id_allocation.gpu_ptr;

        uint32 index = 0;
        for (auto [entity_handle, transform, mesh, anim] : group.each())
        {
            std::lock_guard<std::mutex> lock{mesh.mesh->mutex};

            if (!mesh.mesh || mesh.is_hidden || (mesh.mesh->load_state.load() != AssetLoadState::LOADED))
                continue;

            const trs& world_transform = transform.get_world_transform();
            transforms[index] = trs_to_mat4(world_transform);
            prev_frame_transforms[index] = mesh.update_frame_transform(transforms[index], dx_context.get_frame_id());
            object_ids[index] = (uint32)entity_handle;

            D3D12_GPU_VIRTUAL_ADDRESS base_m = transforms_address + (index * sizeof(mat4));
            D3D12_GPU_VIRTUAL_ADDRESS prev_base_m = prev_frame_transforms_address + (index * sizeof(mat4));
            D3D12_GPU_VIRTUAL_ADDRESS base_object_id = object_id_address + (index * sizeof(uint32));

            const DXMesh& dx_mesh = mesh.mesh->mesh;

            PBRRenderData data;
            data.transform_ptr = base_m;
            data.vertex_buffer = anim.current_vertex_buffer;
            data.index_buffer = dx_mesh.index_buffer;
            data.num_instances = 1;

            DepthPrepassData depth_prepass_data;
            depth_prepass_data.transform_ptr = base_m;
            depth_prepass_data.prev_frame_transform_ptr = prev_base_m;
            depth_prepass_data.object_id_ptr = base_object_id;
            depth_prepass_data.vertex_buffer = anim.current_vertex_buffer;
            depth_prepass_data.prev_frame_vertex_buffer = anim.prev_frame_vertex_buffer.positions ? anim.prev_frame_vertex_buffer.positions : anim.current_vertex_buffer.positions;
            depth_prepass_data.index_buffer = dx_mesh.index_buffer;
            depth_prepass_data.num_instances = 1;

            for (auto& sm : mesh.mesh->submeshes)
            {
                if (!sm.is_visible)
                {
                    continue;
                }
                data.submesh = sm.info;
                data.material = sm.material;

                data.submesh.base_vertex -= mesh.mesh->submeshes[0].info.base_vertex; // Vertex buffer from skinning already points to first vertex.

                depth_prepass_data.submesh = data.submesh;
                depth_prepass_data.alpha_cutout_texture_srv = (sm.material && sm.material->albedo) ? sm.material->albedo->default_srv : DXCPUDescriptorHandle{};

                add_to_render_pass(sm.material->shader, data, depth_prepass_data, opaque_render_pass, transparent_render_pass);

                ShadowRenderData shadow_data;
                shadow_data.transform_ptr = base_m;
                shadow_data.vertex_buffer = anim.current_vertex_buffer.positions;
                shadow_data.index_buffer = dx_mesh.index_buffer;
                shadow_data.submesh = data.submesh;
                shadow_data.num_instances = 1;

                for (uint32 i = 0; i < shadow.num_shadow_render_passes; ++i)
                {
                    auto& pass = shadow.shadow_render_passes[i];
                    add_to_dynamic_render_pass(sm.material->shader, shadow_data, pass.pass, pass.frustum.type == LIGHT_FRUSTUM_SPHERE);
                }

                if (entity_handle == selected_object_id)
                {
                    render_outline(ldr_render_pass, transforms[index], anim.current_vertex_buffer, dx_mesh.index_buffer, data.submesh);
                }
            }

            ++index;
        }
    }

    static void render_terrain(const RenderCamera& camera, World* world, Allocator& arena, Entity::Handle selected_object_id,
        OpaqueRenderPass* opaque_render_pass, TransparentRenderPass* transparent_render_pass, LDRRenderPass* ldr_render_pass, SunShadowRenderPass* sun_shadow_render_pass,
        ComputePass* compute_pass, float dt)
    {
        MemoryMarker temp_memory_marker = arena.get_marker();
        TransformComponent* water_plane_transforms = arena.allocate<TransformComponent>(world->number_of_components_of_type<WaterComponent>());
        uint32 num_water_planes = 0;

        for (auto [entity_handle, water, transform] : world->group(components_group<WaterComponent, TransformComponent>).each())
        {
            const trs& world_transform = transform.get_world_transform();

            water.render(camera, transparent_render_pass, world_transform.position, vec2(world_transform.scale.x, world_transform.scale.z), dt);

            water_plane_transforms[num_water_planes++] = transform;
        }

        for (auto [entity_handle, terrain, position] : world->group(components_group<TerrainComponent, TransformComponent>).each())
        {
            const trs& world_transform = position.get_world_transform();

            terrain.render(camera, opaque_render_pass, sun_shadow_render_pass, ldr_render_pass,
                world_transform.position, selected_object_id == entity_handle, water_plane_transforms, num_water_planes);
        }
        arena.reset_to_marker(temp_memory_marker);

        for (auto [entity_handle, terrain, position, placement] : world->group(components_group<TerrainComponent, TransformComponent, ProcPlacementComponent>).each())
        {
            const trs& world_transform = position.get_world_transform();

            placement.generate(camera, terrain, world_transform.position);
            placement.render(ldr_render_pass);
        }

        for (auto [entity_handle, terrain, position, grass] : world->group(components_group<TerrainComponent, TransformComponent, GrassComponent>).each())
        {
            const trs& world_transform = position.get_world_transform();

            grass.generate(compute_pass, camera, terrain, world_transform.position, dt);
            grass.render(opaque_render_pass);
        }
    }

    static void render_trees(World* world, const CameraFrustumPlanes& frustum, Allocator& arena, Entity::Handle selected_object_id,
        OpaqueRenderPass* opaque_render_pass, TransparentRenderPass* transparent_render_pass, LDRRenderPass* ldr_render_pass, SunShadowRenderPass* sun_shadow_render_pass,
        float dt)
    {
        CPU_PROFILE_BLOCK("Trees");

        auto group = world->group(
            components_group<TransformComponent, MeshComponent, TreeComponent>);

        uint32 group_size = (uint32)group.size();

        if (group_size == 0)
        {
            return;
        }

        std::unordered_map<MultiMesh*, OffsetCount> oc_per_mesh = get_offsets_per_mesh(group);

        DXAllocation transform_allocation = dx_context.allocate_dynamic_buffer(group_size * sizeof(mat4), 4);
        mat4* transforms = (mat4*)transform_allocation.cpu_ptr;

        DXAllocation object_id_allocation = dx_context.allocate_dynamic_buffer(group_size * sizeof(uint32), 4);
        uint32* object_ids = (uint32*)object_id_allocation.cpu_ptr;

        for (auto [entity_handle, transform, mesh, tree] : group.each())
        {
            if (!should_render(frustum, mesh, transform))
                continue;

            const DXMesh& dx_mesh = mesh.mesh->mesh;

            OffsetCount& oc = oc_per_mesh.at(mesh.mesh.get());

            uint32 index = oc.offset + oc.count;

            transforms[index] = trs_to_mat4(transform.get_world_transform());
            object_ids[index] = (uint32)entity_handle;

            ++oc.count;

            if (entity_handle == selected_object_id)
            {
                for (auto& sm : mesh.mesh->submeshes)
                {
                    if (!sm.is_visible)
                    {
                        continue;
                    }
                    render_outline(ldr_render_pass, transforms[index], dx_mesh.vertex_buffer, dx_mesh.index_buffer, sm.info);
                }
            }
        }

        D3D12_GPU_VIRTUAL_ADDRESS transforms_address = transform_allocation.gpu_ptr;
        D3D12_GPU_VIRTUAL_ADDRESS object_id_address = object_id_allocation.gpu_ptr;

        for (auto& [mesh, oc] : oc_per_mesh)
        {
            if (oc.count == 0)
                continue;

            D3D12_GPU_VIRTUAL_ADDRESS base_m = transforms_address + (oc.offset * sizeof(mat4));
            D3D12_GPU_VIRTUAL_ADDRESS base_object_id = object_id_address + (oc.offset * sizeof(uint32));

            render_tree(opaque_render_pass, base_m, oc.count, mesh, dt);
        }
    }

    static void render_cloth(World* world, Entity::Handle selected_object_id,
        OpaqueRenderPass* opaque_render_pass, TransparentRenderPass* transparent_render_pass, LDRRenderPass* ldr_render_pass, SunShadowRenderPass* sun_shadow_render_pass)
    {
        CPU_PROFILE_BLOCK("Cloth");

        auto group = world->group(
            components_group<TransformComponent, ClothRenderComponent>);

        uint32 group_size = (uint32)group.size();

        if (group_size == 0)
        {
            return;
        }

        DXAllocation transform_allocation = dx_context.allocate_dynamic_buffer(1 * sizeof(mat4), 4);
        *(mat4*)transform_allocation.cpu_ptr = mat4::identity;

        DXAllocation object_id_allocation = dx_context.allocate_dynamic_buffer(group_size * sizeof(uint32), 4);
        uint32* object_ids = (uint32*)object_id_allocation.cpu_ptr;

        D3D12_GPU_VIRTUAL_ADDRESS object_id_address = object_id_allocation.gpu_ptr;

        PbrMaterialDesc desc;
        // TODO: change it. Temporal solution
        desc.albedo = get_asset_path("/resources/assets/Sponza/textures/sponza_curtain_diff.png");
        desc.normal = get_asset_path("/resources/assets/Sponza/textures/sponza_fabric_ddn.jpg");
        desc.roughness = get_asset_path("/resources/assets/Sponza/textures/sponza_curtain_diff.png");
        desc.metallic = "";
        desc.shader = PBR_MATERIAL_SHADER_DOUBLE_SIDED;

        static auto cloth_material = create_pbr_material(desc);

        uint32 index = 0;
        for (auto [entity_handle, cloth, render] : group.each())
        {
            object_ids[index] = (uint32)entity_handle;
            D3D12_GPU_VIRTUAL_ADDRESS base_object_id = object_id_address + (index * sizeof(uint32));

            auto [vb, prev_frame_vb, ib, sm] = render.get_render_data();

            PBRRenderData data;
            data.transform_ptr = transform_allocation.gpu_ptr;
            data.vertex_buffer = vb;
            data.index_buffer = ib;
            data.submesh = sm;
            data.material = cloth_material;
            data.num_instances = 1;

            DepthPrepassData depth_prepass_data;
            depth_prepass_data.transform_ptr = transform_allocation.gpu_ptr;
            depth_prepass_data.prev_frame_transform_ptr = transform_allocation.gpu_ptr;
            depth_prepass_data.object_id_ptr = base_object_id;
            depth_prepass_data.vertex_buffer = vb;
            depth_prepass_data.prev_frame_vertex_buffer = prev_frame_vb.positions ? prev_frame_vb.positions : vb.positions;
            depth_prepass_data.index_buffer = ib;
            depth_prepass_data.submesh = sm;
            depth_prepass_data.num_instances = 1;
            depth_prepass_data.alpha_cutout_texture_srv = (cloth_material && cloth_material->albedo) ? cloth_material->albedo->default_srv : DXCPUDescriptorHandle{};

            add_to_render_pass(cloth_material->shader, data, depth_prepass_data, opaque_render_pass, transparent_render_pass);

            if (sun_shadow_render_pass)
            {
                ShadowRenderData shadow_data;
                shadow_data.transform_ptr = transform_allocation.gpu_ptr;
                shadow_data.vertex_buffer = vb.positions;
                shadow_data.index_buffer = ib;
                shadow_data.num_instances = 1;
                shadow_data.submesh = data.submesh;

                add_to_dynamic_render_pass(cloth_material->shader, shadow_data, &sun_shadow_render_pass->cascades[0], false);
            }

            if (entity_handle == selected_object_id)
            {
                render_outline(ldr_render_pass, mat4::identity, vb, ib, sm);
            }

            ++index;
        }
    }

    static void setup_sun_shadow_pass(DirectionalLight& sun, SunShadowRenderPass* sun_shadow_render_pass, bool invalidate_shadow_map_cache)
    {
        ShadowRenderCommand command = determine_sun_shadow_info(sun, invalidate_shadow_map_cache);
        sun_shadow_render_pass->num_cascades = sun.num_shadow_cascades;
        for (uint32 i = 0; i < sun.num_shadow_cascades; ++i)
        {
            SunCascadeRenderPass& cascade_pass = sun_shadow_render_pass->cascades[i];
            cascade_pass.viewport = command.viewports[i];
            cascade_pass.view_proj = sun.view_projs[i];
        }
        sun_shadow_render_pass->copy_from_static_cache = !command.render_static_geometry;
    }

    static void setup_spot_shadow_passes(World* world, SceneLighting& lighting, bool invalidate_shadow_map_cache)
    {
        uint32 num_spot_lights = world->number_of_components_of_type<SpotLightComponent>();
        if (num_spot_lights)
        {
            auto* sl_ptr = (SpotLightCB*)map_buffer(lighting.spot_light_buffer, false);
            auto* si_ptr = (SpotShadowInfo*)map_buffer(lighting.spot_light_shadow_info_buffer, false);

            for (auto [entity_handle, transform, sl] : world->view<TransformComponent, SpotLightComponent>().each())
            {
                const trs& world_transform = transform.get_world_transform();

                SpotLightCB cb(world_transform.position, world_transform.rotation * vec3(0.f, 0.f, -1.f), sl.color * sl.intensity, sl.inner_angle, sl.outer_angle, sl.distance);

                if (sl.casts_shadow && lighting.num_spot_shadow_render_passes < lighting.max_num_spot_shadow_render_passes)
                {
                    cb.shadow_info_index = lighting.num_spot_shadow_render_passes++;

                    auto [command, si] = determine_spot_shadow_info(cb, (uint32)entity_handle, sl.shadow_map_resolution, invalidate_shadow_map_cache);
                    SpotShadowRenderPass& pass = lighting.spot_shadow_render_passes[cb.shadow_info_index];

                    pass.copy_from_static_cache = !command.render_static_geometry;
                    pass.viewport = command.viewports[0];
                    pass.view_proj_matrix = si.view_proj;

                    *si_ptr++ = si;
                }

                *sl_ptr++ = cb;
            }

            unmap_buffer(lighting.spot_light_buffer, true, {0, num_spot_lights});
            unmap_buffer(lighting.spot_light_shadow_info_buffer, true, {0, lighting.num_spot_shadow_render_passes});
        }
    }

    static void setup_point_shadow_passes(World* world, SceneLighting& lighting, bool invalidate_shadow_map_cache)
    {
        uint32 num_point_lights = world->number_of_components_of_type<PointLightComponent>();
        if (num_point_lights)
        {
            auto* pl_ptr = (PointLightCB*)map_buffer(lighting.point_light_buffer, false);
            auto* si_ptr = (PointShadowInfo*)map_buffer(lighting.point_light_shadow_info_buffer, false);

            for (auto [entity_handle, position, pl] : world->view<TransformComponent, PointLightComponent>().each())
            {
                const trs& world_transform = position.get_world_transform();

                PointLightCB cb(world_transform.position, pl.color * pl.intensity, pl.radius);

                if (pl.casts_shadow && lighting.num_point_shadow_render_passes < lighting.max_num_point_shadow_render_passes)
                {
                    cb.shadow_info_index = lighting.num_point_shadow_render_passes++;

                    auto [command, si] = determine_point_shadow_info(cb, (uint32)entity_handle, pl.shadow_map_resolution, invalidate_shadow_map_cache);
                    PointShadowRenderPass& pass = lighting.point_shadow_render_passes[cb.shadow_info_index];

                    pass.copy_from_static_cache0 = !command.render_static_geometry;
                    pass.copy_from_static_cache1 = !command.render_static_geometry;
                    pass.viewport0 = command.viewports[0];
                    pass.viewport1 = command.viewports[1];
                    pass.light_position = cb.position;
                    pass.max_distance = cb.radius;

                    *si_ptr++ = si;
                }

                *pl_ptr++ = cb;
            }

            unmap_buffer(lighting.point_light_buffer, true, {0, num_point_lights});
            unmap_buffer(lighting.point_light_shadow_info_buffer, true, {0, lighting.num_point_shadow_render_passes});
        }
    }

    void render_world(const RenderCamera& camera, World* world, Allocator& arena, Entity::Handle selected_object_id, DirectionalLight& sun, SceneLighting& lighting, bool invalidate_shadow_map_cache, OpaqueRenderPass* opaque_render_pass, TransparentRenderPass* transparent_render_pass, LDRRenderPass* ldr_render_pass, SunShadowRenderPass* sun_shadow_render_pass, ComputePass* compute_pass, float dt)
    {
        CPU_PROFILE_BLOCK("Submit scene render commands");

        setup_sun_shadow_pass(sun, sun_shadow_render_pass, invalidate_shadow_map_cache);
        setup_spot_shadow_passes(world, lighting, invalidate_shadow_map_cache);
        setup_point_shadow_passes(world, lighting, invalidate_shadow_map_cache);

        ShadowPasses static_shadow_passes = {};
        ShadowPasses dynamic_shadow_passes = {};

        MemoryMarker temp_memory_marker = arena.get_marker();
        uint32 num_shadow_casting_lights = 1 + lighting.num_spot_shadow_render_passes + lighting.num_point_shadow_render_passes;
        static_shadow_passes.shadow_render_passes = arena.allocate<ShadowPass>(num_shadow_casting_lights);
        dynamic_shadow_passes.shadow_render_passes = arena.allocate<ShadowPass>(num_shadow_casting_lights);

        {
            auto& out_pass = dynamic_shadow_passes.shadow_render_passes[dynamic_shadow_passes.num_shadow_render_passes++];
            out_pass.frustum.frustum = get_world_space_frustum_planes(sun_shadow_render_pass->cascades[sun_shadow_render_pass->num_cascades - 1].view_proj);
            out_pass.frustum.type = LIGHT_FRUSTUM_STANDARD;
            out_pass.pass = &sun_shadow_render_pass->cascades[0];

            if (!sun_shadow_render_pass->copy_from_static_cache)
            {
                static_shadow_passes.shadow_render_passes[static_shadow_passes.num_shadow_render_passes++] = out_pass;
            }
        }

        for (uint32 i = 0; i < lighting.num_spot_shadow_render_passes; ++i)
        {
            auto pass = &lighting.spot_shadow_render_passes[i];
            auto& out_pass = dynamic_shadow_passes.shadow_render_passes[dynamic_shadow_passes.num_shadow_render_passes++];
            out_pass.frustum.frustum = get_world_space_frustum_planes(pass->view_proj_matrix);
            out_pass.frustum.type = LIGHT_FRUSTUM_STANDARD;
            out_pass.pass = pass;

            if (!pass->copy_from_static_cache)
            {
                static_shadow_passes.shadow_render_passes[static_shadow_passes.num_shadow_render_passes++] = out_pass;
            }
        }

        for (uint32 i = 0; i < lighting.num_point_shadow_render_passes; ++i)
        {
            auto pass = &lighting.point_shadow_render_passes[i];
            auto& out_pass = dynamic_shadow_passes.shadow_render_passes[dynamic_shadow_passes.num_shadow_render_passes++];
            out_pass.frustum.sphere = {pass->light_position, pass->max_distance};
            out_pass.frustum.type = LIGHT_FRUSTUM_SPHERE;
            out_pass.pass = pass;
            out_pass.is_point_light = true;

            if (!pass->copy_from_static_cache0)
            {
                static_shadow_passes.shadow_render_passes[static_shadow_passes.num_shadow_render_passes++] = out_pass;
            }
        }

        bool sun_render_static_geometry = !sun_shadow_render_pass->copy_from_static_cache;

        CameraFrustumPlanes frustum = camera.get_world_space_frustum_planes();

        render_static_objects(world, frustum, arena, selected_object_id, opaque_render_pass, transparent_render_pass, ldr_render_pass, static_shadow_passes);
        render_dynamic_objects(world, frustum, arena, selected_object_id, opaque_render_pass, transparent_render_pass, ldr_render_pass, dynamic_shadow_passes);
        render_animated_objects(world, frustum, arena, selected_object_id, opaque_render_pass, transparent_render_pass, ldr_render_pass, dynamic_shadow_passes);
        render_terrain(camera, world, arena, selected_object_id, opaque_render_pass, transparent_render_pass, ldr_render_pass, sun_render_static_geometry ? sun_shadow_render_pass : 0,
            compute_pass, dt);
        render_trees(world, frustum, arena, selected_object_id, opaque_render_pass, transparent_render_pass, ldr_render_pass, sun_shadow_render_pass, dt);
        render_cloth(world, selected_object_id, opaque_render_pass, transparent_render_pass, ldr_render_pass, sun_shadow_render_pass);

        arena.reset_to_marker(temp_memory_marker);
    }

}