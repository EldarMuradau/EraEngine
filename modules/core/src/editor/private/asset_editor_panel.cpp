// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "editor/asset_editor_panel.h"

#include "core/imgui.h"

#include "geometry/mesh.h"

#include "animation/animation.h"

#include "core/editor_icons.h"
#include "editor/editor.h"

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui/imgui_internal.h>

namespace era_engine
{
    void AssetEditorPanel::begin_frame()
    {
        window_open = window_open_internal;

        if (window_open)
        {
            ImGui::SetNextWindowSize(ImVec2(1280, 800), ImGuiCond_FirstUseEver);
            if (ImGui::Begin(title, &window_open_internal, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
            {
                if (ImGui::BeginTable("##table", 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable))
                {
                    ImGui::TableSetupColumn("ViewportColumn");
                    ImGui::TableSetupColumn("SettingsColumn", ImGuiTableColumnFlags_WidthFixed, 300);

                    ImVec2 content_size;

                    if (ImGui::TableNextColumn())
                    {
                        content_size = ImGui::GetContentRegionAvail();
                        ref<DXTexture> rendering = get_rendering();
                        if (rendering)
                        {
                            ImVec2 min_corner = ImGui::GetCursorPos();
                            ImGui::Image(rendering, (uint32)content_size.x, (uint32)content_size.y);

                            ImGui::SetCursorPos(ImVec2(min_corner.x + 4.5f, min_corner.y + 4.5f));
                            ImGui::Dummy(ImVec2(content_size.x - 9.f, content_size.y - 9.f));

                            if (drag_drop_target && ImGui::BeginDragDropTarget())
                            {
                                if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(drag_drop_target))
                                {
                                    set_drag_drop_data(payload->Data, payload->DataSize);
                                }
                                ImGui::EndDragDropTarget();
                            }
                        }
                    }

                    if (ImGui::TableNextColumn())
                    {
                        edit((uint32)content_size.x, (uint32)content_size.y);
                    }

                    ImGui::EndTable();
                }
            }
            ImGui::End();
        }
    }

    void AssetEditorPanel::open()
    {
        window_open_internal = true;
    }

    void AssetEditorPanel::close()
    {
        window_open_internal = false;
    }

    MeshEditorPanel::MeshEditorPanel()
    {
        title = EDITOR_ICON_MESH "  Mesh Editor";
        drag_drop_target = EDITOR_ICON_MESH;

        RendererSpec spec = {false, false, false, false, false, false};

        renderer.initialize(COLOR_DEPTH_8, 1280, 800, spec);

        camera.initialize_ingame(vec3(0.f, 0.f, 25.f), quat::identity, deg2rad(80.f), 0.01f);

        controller.initialize(&camera);
    }

    void MeshEditorPanel::edit(uint32 render_width, uint32 render_height)
    {
        width = render_width;
        height = render_height;

        renderer.begin_frame(render_width, render_height);

        DirectionalLight& sun = scene->sun;
        PBREnvironment& environment = scene->environment;

        environment.update(sun.direction);

        sun.update_matrices(camera);
        environment.light_probe_grid.visualize(&render_pass);

        renderer.set_environment(environment);
        renderer.set_sun(sun);

        camera.set_viewport(render_width, render_height);
        camera.update_matrices();

        render_pass.reset();

        if (this->mesh)
        {
            DXAllocation transform_allocation = dx_context.allocate_dynamic_buffer(sizeof(mat4), 4);
            DXAllocation object_id_allocation = dx_context.allocate_dynamic_buffer(sizeof(uint32), 4);

            mat4* transform = (mat4*)transform_allocation.cpu_ptr;
            transform[0] = mat4::identity;

            uint32* object_id = (uint32*)object_id_allocation.cpu_ptr;
            object_id[0] = (uint32)0;

            D3D12_GPU_VIRTUAL_ADDRESS transforms_address = transform_allocation.gpu_ptr;
            D3D12_GPU_VIRTUAL_ADDRESS base_object_id = object_id_allocation.gpu_ptr;

            const DXMesh& mesh = this->mesh->mesh;

            PBRRenderData data;
            data.transform_ptr = transforms_address;
            data.vertex_buffer = mesh.vertex_buffer;
            data.index_buffer = mesh.index_buffer;
            data.num_instances = 1;

            DepthPrepassData depth_prepass_data;
            depth_prepass_data.transform_ptr = transforms_address;
            depth_prepass_data.prev_frame_transform_ptr = transforms_address;
            depth_prepass_data.object_id_ptr = base_object_id;
            depth_prepass_data.vertex_buffer = mesh.vertex_buffer;
            depth_prepass_data.prev_frame_vertex_buffer = mesh.vertex_buffer.positions;
            depth_prepass_data.index_buffer = mesh.index_buffer;
            depth_prepass_data.num_instances = 1;

            for (auto& sm : this->mesh->submeshes)
            {
                SubmeshInfo submesh = sm.info;
                const ref<PBRMaterial>& material = sm.material;

                data.material = material;
                data.submesh = submesh;

                depth_prepass_data.submesh = data.submesh;
                depth_prepass_data.alpha_cutout_texture_srv = (sm.material && sm.material->albedo) ? sm.material->albedo->default_srv : DXCPUDescriptorHandle{};

                render_pass.render_object<PBRPipeline::OpaqueDoubleSided>(data);
                render_pass.render_depth_only<DepthPrepassPipeline::SingleSided>(depth_prepass_data);
            }
        }

        renderer.submit_render_pass(&render_pass);
        renderer.set_camera(camera);

        if (ImGui::BeginTabBar("Tabs"))
        {
            if (ImGui::BeginTabItem("Geometry"))
            {
                if (this->mesh && ImGui::BeginChild("GeometrySettings"))
                {
                    if (ImGui::BeginProperties())
                    {
                        ImGui::PropertyValue("Load state", asset_load_state_names[mesh->load_state.load()]);
                        edit_mesh("Mesh", mesh, MESH_CREATION_FLAGS_DEFAULT);
                        ImGui::EndProperties();
                    }
                    if (ImGui::BeginTree("Childs"))
                    {
                        for (auto& sub : mesh->submeshes)
                        {
                            ImGui::PushID(&sub);
                            if (ImGui::BeginTree(sub.name.c_str()))
                            {
                                edit_material(sub.material);
                                ImGui::EndTree();
                            }
                            ImGui::PopID();
                        }

                        ImGui::EndTree();
                    }

                    ImGui::EndChild();
                }

                ImGui::EndTabItem();
            }

            /*using namespace era_engine::animation;

            if (this->mesh && ImGui::BeginTabItem("Animations"))
            {
                if (ImGui::BeginChild("AnimationSettings"))
                {
                    Skeleton& skeleton = this->mesh->skeleton;
                    if (skeleton.joints.size() > 0)
                    {
                        if (ImGui::BeginTree("Skeleton"))
                        {
                            if (ImGui::BeginTree("Limbs"))
                            {
                                for (uint32 i = 0; i < limb_type_count; ++i)
                                {
                                    if (i != limb_type_none)
                                    {
                                        SkeletonLimb& l = skeleton.limbs[i];
                                        vec3 c = limb_type_colors[i];
                                        if (ImGui::BeginTreeColoredText(limb_type_names[i], c))
                                        {
                                            if (ImGui::BeginProperties())
                                            {
                                                LimbDimensions& d = l.dimensions;
                                                ImGui::PropertyDrag("Min Y", d.minY, 0.01f);
                                                ImGui::PropertyDrag("Max Y", d.maxY, 0.01f);
                                                ImGui::PropertyDrag("Radius", d.radius, 0.01f);

                                                ImGui::PropertyDrag("Offset X", d.xOffset, 0.01f);
                                                ImGui::PropertyDrag("Offset Z", d.zOffset, 0.01f);

                                                ImGui::EndProperties();
                                            }

                                            ImGui::EndTree();
                                        }
                                    }
                                }

                                ImGui::EndTree();
                            }

                            if (ImGui::BeginTree("Joints"))
                            {
                                for (uint32 i = 0; i < (uint32)skeleton.joints.size(); ++i)
                                {
                                    const SkeletonJoint& j = skeleton.joints[i];
                                    vec3 c = limb_type_colors[j.limb_type];
                                    ImGui::TextColored(ImVec4(c.x, c.y, c.z, 1.f), j.name.c_str());
                                }

                                ImGui::EndTree();
                            }

                            ImGui::EndTree();
                        }
                    }

                    ImGui::EndChild();
                }

                ImGui::EndTabItem();
            }
            */
            ImGui::EndTabBar();
        }
    }

    void MeshEditorPanel::begin_frame()
    {
        window_open = window_open_internal;

        if (window_open)
        {
            ImGui::SetNextWindowSize(ImVec2(1280, 800), ImGuiCond_FirstUseEver);
            if (ImGui::Begin(title, &window_open_internal, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
            {
                if (ImGui::BeginTable("##table", 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable))
                {
                    ImGui::TableSetupColumn("ViewportColumn");
                    ImGui::TableSetupColumn("SettingsColumn", ImGuiTableColumnFlags_WidthFixed, 300);

                    ImVec2 content_size;

                    if (ImGui::TableNextColumn())
                    {
                        content_size = ImGui::GetContentRegionAvail();
                        ref<DXTexture> rendering = get_rendering();
                        if (rendering)
                        {
                            ImVec2 min_corner = ImGui::GetCursorPos();
                            ImGui::Image(rendering, (uint32)content_size.x, (uint32)content_size.y);

                            if (ImGui::IsItemHovered())
                            {
                                hovered = true;
                                ImVec2 relative_mouse = ImGui::GetMousePos() - ImGui::GetItemRectMin();
                                vec2 mouse_pos = {relative_mouse.x, relative_mouse.y};
                                if (app_focused_last_frame)
                                {
                                    input.mouse.dx = (int32)(mouse_pos.x - input.mouse.x);
                                    input.mouse.dy = (int32)(mouse_pos.y - input.mouse.y);
                                    input.mouse.reldx = (float)input.mouse.dx / (width - 1);
                                    input.mouse.reldy = (float)input.mouse.dy / (height - 1);
                                }
                                else
                                {
                                    input.mouse.dx = 0;
                                    input.mouse.dy = 0;
                                    input.mouse.reldx = 0.f;
                                    input.mouse.reldy = 0.f;
                                }
                                input.mouse.x = (int32)mouse_pos.x;
                                input.mouse.y = (int32)mouse_pos.y;
                                input.mouse.rel_x = mouse_pos.x / (width - 1);
                                input.mouse.rel_y = mouse_pos.y / (height - 1);
                                input.mouse.left = {ImGui::IsMouseDown(ImGuiMouseButton_Left), ImGui::IsMouseClicked(ImGuiMouseButton_Left), ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)};
                                input.mouse.right = {ImGui::IsMouseDown(ImGuiMouseButton_Right), ImGui::IsMouseClicked(ImGuiMouseButton_Right), ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Right)};
                                input.mouse.middle = {ImGui::IsMouseDown(ImGuiMouseButton_Middle), ImGui::IsMouseClicked(ImGuiMouseButton_Middle), ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Middle)};

                                for (uint32 i = 0; i < arraysize(UserInput::keyboard); ++i)
                                {
                                    input.keyboard[i] = {ImGui::IsKeyDown(i), ImGui::IsKeyPressed(i, false)};
                                }

                                input.over_window = true;
                            }
                            else
                            {
                                hovered = false;
                                input.mouse.dx = 0;
                                input.mouse.dy = 0;
                                input.mouse.reldx = 0.f;
                                input.mouse.reldy = 0.f;

                                if (input.mouse.left.down && !ImGui::IsMouseDown(ImGuiMouseButton_Left))
                                {
                                    input.mouse.left.down = false;
                                }
                                if (input.mouse.right.down && !ImGui::IsMouseDown(ImGuiMouseButton_Right))
                                {
                                    input.mouse.right.down = false;
                                }
                                if (input.mouse.middle.down && !ImGui::IsMouseDown(ImGuiMouseButton_Middle))
                                {
                                    input.mouse.middle.down = false;
                                }

                                input.mouse.left.click_event = input.mouse.left.double_click_event = false;
                                input.mouse.right.click_event = input.mouse.right.double_click_event = false;
                                input.mouse.middle.click_event = input.mouse.middle.double_click_event = false;

                                for (uint32 i = 0; i < arraysize(UserInput::keyboard); ++i)
                                {
                                    if (!ImGui::IsKeyDown(i))
                                    {
                                        input.keyboard[i].down = false;
                                    }
                                    input.keyboard[i].press_event = false;
                                }

                                input.over_window = false;
                            }

                            ImGui::SetCursorPos(ImVec2(min_corner.x + 4.5f, min_corner.y + 4.5f));
                            ImGui::Dummy(ImVec2(content_size.x - 9.f, content_size.y - 9.f));

                            if (drag_drop_target && ImGui::BeginDragDropTarget())
                            {
                                if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(drag_drop_target))
                                {
                                    set_drag_drop_data(payload->Data, payload->DataSize);
                                }
                                ImGui::EndDragDropTarget();
                            }
                        }
                    }

                    if (ImGui::TableNextColumn())
                    {
                        edit((uint32)content_size.x, (uint32)content_size.y);
                    }

                    ImGui::EndTable();
                }
            }
            ImGui::End();
        }
    }

    void MeshEditorPanel::end_frame()
    {
        if (is_open())
        {
            renderer.end_collection();
            renderer.end_frame(&input);
            controller.update(input, width, height, 1.0f / 10.0f);
        }
    }

    ref<DXTexture> MeshEditorPanel::get_rendering()
    {
        return renderer.frame_result;
    }

    void MeshEditorPanel::set_drag_drop_data(void* data, uint32 size)
    {
        const char* filename = (const char*)data;

        fs::path path = filename;
        fs::path relative = fs::relative(path, fs::current_path());

        this->mesh = import_mesh_from_file(relative.string());
    }
}