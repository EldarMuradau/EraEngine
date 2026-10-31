// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "animation/skinning.h"

#include "dx/dx_command_list.h"
#include "dx/dx_barrier_batcher.h"
#include "dx/dx_profiling.h"
#include "dx/dx_buffer.h"

#include "rendering/material.h"
#include "rendering/render_pass.h"

#include "skinning_rs.hlsli"

namespace era_engine::animation
{
    static DXPipeline skinning_pipeline;
    static DXPipeline cloth_skinning_pipeline;

    struct SkinningCall
    {
        DXVertexBufferGroupView vertex_buffer;
        VertexRange range;
        uint32 joint_offset;
        uint32 num_joints;
        uint32 vertex_offset;
    };

    struct ClothSkinningCall
    {
        DXVertexBufferView vertex_buffer;
        uint32 grid_size_x;
        uint32 grid_size_y;
        uint32 vertex_offset;
    };

    struct SkinningData
    {
        VertexBufferGroup skinned_vertex_buffer;
        ref<DXBuffer> skinning_matrices_buffer;

        mat4* skinning_matrices;
        uint32 num_skinning_matrices;
        uint32 matrix_offset;

        SkinningCall* calls;
        uint32 num_calls;

        ClothSkinningCall* cloth_calls;
        uint32 num_cloth_calls;
    };

    struct SkinningPipeline
    {
        PIPELINE_SETUP_DECL {}

        PIPELINE_COMPUTE_DECL(SkinningData)
        {
            PROFILE_ALL(cl, "Skinning");

            if (data.num_calls)
            {
                PROFILE_ALL(cl, "Skeletal");

                mat4* mats = (mat4*)map_buffer(data.skinning_matrices_buffer, false);
                memcpy(mats + data.matrix_offset, data.skinning_matrices, sizeof(mat4) * data.num_skinning_matrices);
                unmap_buffer(data.skinning_matrices_buffer, true, MapRange{data.matrix_offset, data.num_skinning_matrices});

                cl->set_pipeline_state(*skinning_pipeline.pipeline);
                cl->set_compute_root_signature(*skinning_pipeline.root_signature);

                cl->set_root_compute_srv(SKINNING_RS_MATRICES, data.skinning_matrices_buffer->gpu_virtual_address + sizeof(mat4) * data.matrix_offset);
                cl->set_root_compute_uav(SKINNING_RS_OUTPUT0, data.skinned_vertex_buffer.positions);
                cl->set_root_compute_uav(SKINNING_RS_OUTPUT1, data.skinned_vertex_buffer.others);

                for (uint32 i = 0; i < data.num_calls; ++i)
                {
                    auto& c = data.calls[i];
                    cl->set_root_compute_srv(SKINNING_RS_INPUT_VERTEX_BUFFER0, c.vertex_buffer.positions.view.BufferLocation);
                    cl->set_root_compute_srv(SKINNING_RS_INPUT_VERTEX_BUFFER1, c.vertex_buffer.others.view.BufferLocation);
                    cl->set_compute_32bit_constants(SKINNING_RS_CB, SkinningCB{c.joint_offset, c.num_joints, c.range.first_vertex, c.range.num_vertices, c.vertex_offset});
                    cl->dispatch(bucketize(c.range.num_vertices, 512));
                }
            }
            if (data.num_cloth_calls)
            {
                PROFILE_ALL(cl, "Cloth");

                cl->set_pipeline_state(*cloth_skinning_pipeline.pipeline);
                cl->set_compute_root_signature(*cloth_skinning_pipeline.root_signature);

                cl->set_root_compute_uav(CLOTH_SKINNING_RS_OUTPUT0, data.skinned_vertex_buffer.positions);
                cl->set_root_compute_uav(CLOTH_SKINNING_RS_OUTPUT1, data.skinned_vertex_buffer.others);

                for (uint32 i = 0; i < data.num_cloth_calls; ++i)
                {
                    auto& c = data.cloth_calls[i];
                    cl->set_root_compute_srv(CLOTH_SKINNING_RS_INPUT, c.vertex_buffer.view.BufferLocation);
                    cl->set_compute_32bit_constants(CLOTH_SKINNING_RS_CB, ClothSkinningCB{c.grid_size_x, c.grid_size_y, c.vertex_offset});
                    cl->dispatch(bucketize(c.grid_size_x, 15), bucketize(c.grid_size_y, 15)); // 15 is correct here.
                }
            }
        }
    };

#define MAX_NUM_SKINNING_MATRICES_PER_FRAME 4096
#define MAX_NUM_SKINNED_VERTICES_PER_FRAME (1024 * 256)

    static ref<DXBuffer> skinning_matrices_buffer; // Buffered frames are in a single dx_buffer.

    // Skinning runs on the async compute queue, which does not wait for the render queue, so a frame writes its buffer
    // while the previous frames may still render from theirs (current and previous for screen space velocities).
    static constexpr uint32 NUM_SKINNED_VERTEX_BUFFERS = NUM_BUFFERED_FRAMES + 1;

    static uint32 current_skinned_vertex_buffer;
    static VertexBufferGroup skinned_vertex_buffer[NUM_SKINNED_VERTEX_BUFFERS];

    // Per frame slot. The execution of a frame copies its matrices and calls while the next frame collects.
    static mat4 skinning_matrices[NUM_BUFFERED_FRAMES][MAX_NUM_SKINNING_MATRICES_PER_FRAME];
    static volatile uint32 num_skinning_matrices_this_frame;

    static SkinningCall calls[NUM_BUFFERED_FRAMES][1024];
    static volatile uint32 num_calls;

    static ClothSkinningCall cloth_calls[NUM_BUFFERED_FRAMES][128];
    static volatile uint32 num_cloth_calls;

    static volatile uint32 total_num_vertices;

    ERA_CORE_API void initialize_skinning()
    {
        skinning_matrices_buffer = create_upload_buffer(sizeof(mat4), MAX_NUM_SKINNING_MATRICES_PER_FRAME * NUM_BUFFERED_FRAMES, 0);

        for (uint32 i = 0; i < NUM_SKINNED_VERTEX_BUFFERS; ++i)
        {
            skinned_vertex_buffer[i].positions = create_vertex_buffer(sizeof(vec3), MAX_NUM_SKINNED_VERTICES_PER_FRAME, 0, true);
            skinned_vertex_buffer[i].others = create_vertex_buffer(
                get_vertex_others_size(MESH_CREATION_FLAGS_WITH_POSITIONS | MESH_CREATION_FLAGS_WITH_UVS | MESH_CREATION_FLAGS_WITH_NORMALS | MESH_CREATION_FLAGS_WITH_TANGENTS),
                MAX_NUM_SKINNED_VERTICES_PER_FRAME, 0, true);
        }

        skinning_pipeline = create_reloadable_pipeline("skinning_cs");
        cloth_skinning_pipeline = create_reloadable_pipeline("cloth_skinning_cs");
    }

    ERA_CORE_API std::tuple<DXVertexBufferGroupView, mat4*> skin_object(const DXVertexBufferGroupView& vertex_buffer, VertexRange range, uint32 num_joints)
    {
        uint32 joint_offset = atomic_add(num_skinning_matrices_this_frame, num_joints);
        ASSERT(joint_offset + num_joints <= MAX_NUM_SKINNING_MATRICES_PER_FRAME);

        uint32 vertex_offset = atomic_add(total_num_vertices, range.num_vertices);
        ASSERT(vertex_offset + range.num_vertices <= MAX_NUM_SKINNED_VERTICES_PER_FRAME);

        const uint32 slot = dx_context.get_buffered_frame_id();

        uint32 call_index = atomic_increment(num_calls);
        ASSERT(call_index < arraysize(calls[slot]));

        calls[slot][call_index] =
            {
                vertex_buffer,
                range,
                joint_offset,
                num_joints,
                vertex_offset};

        uint32 num_vertices = range.num_vertices;

        auto& positions = skinned_vertex_buffer[current_skinned_vertex_buffer].positions;
        auto& others = skinned_vertex_buffer[current_skinned_vertex_buffer].others;

        DXVertexBufferGroupView result;
        result.positions.view.BufferLocation = positions->gpu_virtual_address + positions->element_size * vertex_offset;
        result.positions.view.SizeInBytes = positions->element_size * num_vertices;
        result.positions.view.StrideInBytes = positions->element_size;
        result.others.view.BufferLocation = others->gpu_virtual_address + others->element_size * vertex_offset;
        result.others.view.SizeInBytes = others->element_size * num_vertices;
        result.others.view.StrideInBytes = others->element_size;

        return {result, skinning_matrices[slot] + joint_offset};
    }

    ERA_CORE_API std::tuple<DXVertexBufferGroupView, mat4*> skin_object(const DXVertexBufferGroupView& vertex_buffer, uint32 num_vertices, uint32 num_joints)
    {
        auto [vb, mats] = skin_object(vertex_buffer, VertexRange{0, num_vertices}, num_joints);
        return {vb, mats};
    }

    ERA_CORE_API std::tuple<DXVertexBufferGroupView, mat4*> skin_object(const DXVertexBufferGroupView& vertex_buffer, SubmeshInfo submesh, uint32 num_joints)
    {
        auto [vb, mats] = skin_object(vertex_buffer, VertexRange{submesh.base_vertex, submesh.num_vertices}, num_joints);

        return {vb, mats};
    }

    ERA_CORE_API DXVertexBufferGroupView skin_cloth(const DXVertexBufferView& inpositions, uint32 grid_size_x, uint32 grid_size_y)
    {
        uint32 num_vertices = grid_size_x * grid_size_y;
        uint32 vertex_offset = atomic_add(total_num_vertices, num_vertices);
        ASSERT(vertex_offset + num_vertices <= MAX_NUM_SKINNED_VERTICES_PER_FRAME);

        const uint32 slot = dx_context.get_buffered_frame_id();

        uint32 call_index = atomic_increment(num_cloth_calls);
        ASSERT(call_index < arraysize(cloth_calls[slot]));

        cloth_calls[slot][call_index] =
            {
                inpositions,
                grid_size_x,
                grid_size_y,
                vertex_offset};

        auto& positions = skinned_vertex_buffer[current_skinned_vertex_buffer].positions;
        auto& others = skinned_vertex_buffer[current_skinned_vertex_buffer].others;

        DXVertexBufferGroupView result;
        result.positions.view.BufferLocation = positions->gpu_virtual_address + positions->element_size * vertex_offset;
        result.positions.view.SizeInBytes = positions->element_size * num_vertices;
        result.positions.view.StrideInBytes = positions->element_size;
        result.others.view.BufferLocation = others->gpu_virtual_address + others->element_size * vertex_offset;
        result.others.view.SizeInBytes = others->element_size * num_vertices;
        result.others.view.StrideInBytes = others->element_size;
        return result;
    }

    ERA_CORE_API void perform_skinning(ComputePass* compute_pass)
    {
        // TODO: We currently make no attempt to ensure that all draw calls have actually been written completely, if executed on another thread.
        if (num_calls > 0 || num_cloth_calls > 0)
        {
            const uint32 slot = dx_context.get_buffered_frame_id();

            SkinningData data;
            data.skinned_vertex_buffer = skinned_vertex_buffer[current_skinned_vertex_buffer];
            data.skinning_matrices_buffer = skinning_matrices_buffer;

            data.skinning_matrices = skinning_matrices[slot];
            data.num_skinning_matrices = num_skinning_matrices_this_frame;
            data.matrix_offset = slot * MAX_NUM_SKINNING_MATRICES_PER_FRAME;

            data.calls = calls[slot];
            data.num_calls = num_calls;

            data.cloth_calls = cloth_calls[slot];
            data.num_cloth_calls = num_cloth_calls;

            compute_pass->add_task<SkinningPipeline>(COMPUTE_PASS_FRAME_START, data);

            current_skinned_vertex_buffer = (current_skinned_vertex_buffer + 1) % NUM_SKINNED_VERTEX_BUFFERS;
            num_calls = 0;
            num_cloth_calls = 0;
            num_skinning_matrices_this_frame = 0;
            total_num_vertices = 0;
        }
    }
}