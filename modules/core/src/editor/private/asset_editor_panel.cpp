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
	void asset_editor_panel::beginFrame()
	{
		windowOpen = windowOpenInternal;

		if (windowOpen)
		{
			ImGui::SetNextWindowSize(ImVec2(1280, 800), ImGuiCond_FirstUseEver);
			if (ImGui::Begin(title, &windowOpenInternal, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
			{
				if (ImGui::BeginTable("##table", 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable))
				{
					ImGui::TableSetupColumn("ViewportColumn");
					ImGui::TableSetupColumn("SettingsColumn", ImGuiTableColumnFlags_WidthFixed, 300);

					ImVec2 contentSize;

					if (ImGui::TableNextColumn())
					{
						contentSize = ImGui::GetContentRegionAvail();
						ref<dx_texture> rendering = getRendering();
						if (rendering)
						{
							ImVec2 minCorner = ImGui::GetCursorPos();
							ImGui::Image(rendering, (uint32)contentSize.x, (uint32)contentSize.y);

							ImGui::SetCursorPos(ImVec2(minCorner.x + 4.5f, minCorner.y + 4.5f));
							ImGui::Dummy(ImVec2(contentSize.x - 9.f, contentSize.y - 9.f));

							if (dragDropTarget && ImGui::BeginDragDropTarget())
							{
								if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(dragDropTarget)) { setDragDropData(payload->Data, payload->DataSize); }
								ImGui::EndDragDropTarget();
							}
						}
					}

					if (ImGui::TableNextColumn())
					{
						edit((uint32)contentSize.x, (uint32)contentSize.y);
					}

					ImGui::EndTable();
				}
			}
			ImGui::End();
		}
	}

	void asset_editor_panel::open()
	{
		windowOpenInternal = true;
	}

	void asset_editor_panel::close()
	{
		windowOpenInternal = false;
	}

	mesh_editor_panel::mesh_editor_panel()
	{
		title = EDITOR_ICON_MESH "  Mesh Editor";
		dragDropTarget = EDITOR_ICON_MESH;

		renderer_spec spec = { false, false, false, false, false, false };

		renderer.initialize(color_depth_8, 1280, 800, spec);

		camera.initializeIngame(vec3(0.f, 0.f, 25.f), quat::identity, deg2rad(80.f), 0.01f);

		controller.initialize(&camera);
	}

	void mesh_editor_panel::edit(uint32 renderWidth, uint32 renderHeight)
	{
		width = renderWidth;
		height = renderHeight;

		renderer.beginFrame(renderWidth, renderHeight);

		directional_light& sun = scene->sun;
		pbr_environment& environment = scene->environment;

		environment.update(sun.direction);

		sun.updateMatrices(camera);
		environment.lightProbeGrid.visualize(&renderPass);

		renderer.setEnvironment(environment);
		renderer.setSun(sun);

		camera.setViewport(renderWidth, renderHeight);
		camera.updateMatrices();

		renderPass.reset();

		if (this->mesh)
		{
			dx_allocation transformAllocation = dxContext.allocateDynamicBuffer(sizeof(mat4), 4);
			dx_allocation objectIDAllocation = dxContext.allocateDynamicBuffer(sizeof(uint32), 4);

			mat4* transform = (mat4*)transformAllocation.cpuPtr;
			transform[0] = mat4::identity;

			uint32* objectID = (uint32*)objectIDAllocation.cpuPtr;
			objectID[0] = (uint32)0;

			D3D12_GPU_VIRTUAL_ADDRESS transformsAddress = transformAllocation.gpuPtr;
			D3D12_GPU_VIRTUAL_ADDRESS baseObjectID = objectIDAllocation.gpuPtr;

			const dx_mesh& mesh = this->mesh->mesh;

			pbr_render_data data;
			data.transformPtr = transformsAddress;
			data.vertexBuffer = mesh.vertexBuffer;
			data.indexBuffer = mesh.indexBuffer;
			data.numInstances = 1;

			depth_prepass_data depthPrepassData;
			depthPrepassData.transformPtr = transformsAddress;
			depthPrepassData.prevFrameTransformPtr = transformsAddress;
			depthPrepassData.objectIDPtr = baseObjectID;
			depthPrepassData.vertexBuffer = mesh.vertexBuffer;
			depthPrepassData.prevFrameVertexBuffer = mesh.vertexBuffer.positions;
			depthPrepassData.indexBuffer = mesh.indexBuffer;
			depthPrepassData.numInstances = 1;

			for (auto& sm : this->mesh->submeshes)
			{
				submesh_info submesh = sm.info;
				const ref<pbr_material>& material = sm.material;

				data.material = material;
				data.submesh = submesh;

				depthPrepassData.submesh = data.submesh;
				depthPrepassData.alphaCutoutTextureSRV = (sm.material && sm.material->albedo) ? sm.material->albedo->defaultSRV : dx_cpu_descriptor_handle{};

				renderPass.renderObject<pbr_pipeline::opaque_double_sided>(data);
				renderPass.renderDepthOnly<depth_prepass_pipeline::single_sided>(depthPrepassData);
			}
		}

		renderer.submitRenderPass(&renderPass);
		renderer.setCamera(camera);

		if (ImGui::BeginTabBar("Tabs"))
		{
			if (ImGui::BeginTabItem("Geometry"))
			{
				if (this->mesh && ImGui::BeginChild("GeometrySettings"))
				{
					if (ImGui::BeginProperties())
					{
						ImGui::PropertyValue("Load state", asset_load_state_names[mesh->loadState.load()]);
						editMesh("Mesh", mesh, mesh_creation_flags_default);
						ImGui::EndProperties();
					}
					if (ImGui::BeginTree("Childs"))
					{
						for (auto& sub : mesh->submeshes)
						{
							ImGui::PushID(&sub);
							if (ImGui::BeginTree(sub.name.c_str()))
							{
								editMaterial(sub.material);
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

	void mesh_editor_panel::beginFrame()
	{
		windowOpen = windowOpenInternal;

		if (windowOpen)
		{
			ImGui::SetNextWindowSize(ImVec2(1280, 800), ImGuiCond_FirstUseEver);
			if (ImGui::Begin(title, &windowOpenInternal, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
			{
				if (ImGui::BeginTable("##table", 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable))
				{
					ImGui::TableSetupColumn("ViewportColumn");
					ImGui::TableSetupColumn("SettingsColumn", ImGuiTableColumnFlags_WidthFixed, 300);

					ImVec2 contentSize;

					if (ImGui::TableNextColumn())
					{
						contentSize = ImGui::GetContentRegionAvail();
						ref<dx_texture> rendering = getRendering();
						if (rendering)
						{
							ImVec2 minCorner = ImGui::GetCursorPos();
							ImGui::Image(rendering, (uint32)contentSize.x, (uint32)contentSize.y);

							if (ImGui::IsItemHovered())
							{
								hovered = true;
								ImVec2 relativeMouse = ImGui::GetMousePos() - ImGui::GetItemRectMin();
								vec2 mousePos = { relativeMouse.x, relativeMouse.y };
								if (appFocusedLastFrame)
								{
									input.mouse.dx = (int32)(mousePos.x - input.mouse.x);
									input.mouse.dy = (int32)(mousePos.y - input.mouse.y);
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
								input.mouse.x = (int32)mousePos.x;
								input.mouse.y = (int32)mousePos.y;
								input.mouse.relX = mousePos.x / (width - 1);
								input.mouse.relY = mousePos.y / (height - 1);
								input.mouse.left = { ImGui::IsMouseDown(ImGuiMouseButton_Left), ImGui::IsMouseClicked(ImGuiMouseButton_Left), ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) };
								input.mouse.right = { ImGui::IsMouseDown(ImGuiMouseButton_Right), ImGui::IsMouseClicked(ImGuiMouseButton_Right), ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Right) };
								input.mouse.middle = { ImGui::IsMouseDown(ImGuiMouseButton_Middle), ImGui::IsMouseClicked(ImGuiMouseButton_Middle), ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Middle) };

								for (uint32 i = 0; i < arraysize(UserInput::keyboard); ++i)
								{
									input.keyboard[i] = { ImGui::IsKeyDown(i), ImGui::IsKeyPressed(i, false) };
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

							ImGui::SetCursorPos(ImVec2(minCorner.x + 4.5f, minCorner.y + 4.5f));
							ImGui::Dummy(ImVec2(contentSize.x - 9.f, contentSize.y - 9.f));

							if (dragDropTarget && ImGui::BeginDragDropTarget())
							{
								if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(dragDropTarget)) { setDragDropData(payload->Data, payload->DataSize); }
								ImGui::EndDragDropTarget();
							}
						}
					}

					if (ImGui::TableNextColumn())
					{
						edit((uint32)contentSize.x, (uint32)contentSize.y);
					}

					ImGui::EndTable();
				}
			}
			ImGui::End();
		}
	}

	void mesh_editor_panel::endFrame()
	{
		if (isOpen())
		{
			renderer.endFrame(&input);
			controller.update(input, width, height, 1.0f / 10.0f);
		}
	}

	ref<dx_texture> mesh_editor_panel::getRendering()
	{
		return renderer.frameResult;
	}

	void mesh_editor_panel::setDragDropData(void* data, uint32 size)
	{
		const char* filename = (const char*)data;

		fs::path path = filename;
		fs::path relative = fs::relative(path, fs::current_path());

		this->mesh = loadMeshFromFile(relative.string());
	}
}