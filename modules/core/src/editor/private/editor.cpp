// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "editor/editor.h"
#include "editor/system_calls.h"

#include "core/cpu_profiling.h"
#include "core/log.h"

#include "dx/dx_profiling.h"

#include "animation/animation.h"

#include "geometry/mesh.h"

#include "audio/audio.h"

#include "rendering/debug_visualization.h"

#include "terrain/terrain.h"
#include "terrain/proc_placement.h"
#include "terrain/grass.h"
#include "terrain/water.h"

#include "imgui/imgui_internal.h"

//#include "scripting/script.h"
//#include "scripting/native_scripting_linker.h"

#include "ecs/base_components/base_components.h"
#include "ecs/world.h"
#include "ecs/rendering/mesh_component.h"
#include "ecs/editor/entity_editor_utils.h"

//#include "physics/core/physics.h"
//#include "physics/body_component.h"
//#include "physics/shape_component.h"

#include <fontawesome/list.h>

#include <sstream>

namespace era_engine
{
	template <typename Component_, typename Member_>
	struct component_member_undo
	{
		component_member_undo(Member_& member, Member_ before, Entity entity, void (*callback)(Component_&, Member_, void*) = 0, void* userData = 0)
		{
			this->entity = entity;
			this->byteOffset = (uint8*)&member - (uint8*)&entity.get_component<Component_>();
			this->before = before;
			this->callback = callback;
			this->userData = userData;
		}

		void toggle()
		{
			if (entity.valid())
			{
				Component_& comp = entity.get_component<Component_>();
				auto& member = *(Member_*)((uint8*)&comp + byteOffset);
				std::swap(member, before);

				if (callback)
				{
					callback(comp, member, userData);
				}
			}
		}

	private:
		Entity entity;
		uint64 byteOffset;

		Member_ before;

		void (*callback)(Component_&, Member_, void*) = 0;
		void* userData = 0;
	};

	template <typename... Component_>
	struct component_undo
	{
		component_undo(Entity entity, Component_... before)
		{
			this->entity = entity;
			this->before = std::make_tuple(before...);
		}

		void toggle()
		{
			if (entity.valid())
			{
				std::apply([this](auto&... c) { (set(c), ...); }, before);
			}
		}

	private:
		template <typename T>
		void set(T& c)
		{
			std::swap(entity.get_component<T>(), c);
		}

		Entity entity;

		std::tuple<Component_...> before;
	};

	template <typename Value_>
	struct settings_undo
	{
		settings_undo(Value_& value, Value_ before)
			: value(value), before(before) {}

		void toggle() { std::swap(value, before); }

	private:
		Value_& value;

		Value_ before;
	};

	struct entity_existence_undo
	{
		// TODO
		entity_existence_undo(ref<World> _world, Entity _entity)
			: world(_world), entity(_entity)
		{
			//size = serializeEntityToMemory(entity, buffer, sizeof(buffer));
		}

		~entity_existence_undo() {}

		void toggle()
		{
			//if (entity.is_valid()) { deleteEntity(); }
			//else { restoreEntity(); }
		}

	private:
		void deleteEntity()
		{
			//size = serializeEntityToMemory(entity, buffer, sizeof(buffer));
			//world->destroy_entity(entity);
		}

		void restoreEntity()
		{
			//Entity::Handle place = entity.get_handle();
			//entity = world->try_create_entity_in_place(entity, "");
			//ASSERT(entity.get_handle() == place);

			//bool success = deserializeEntityFromMemory(entity, buffer, size);
			//ASSERT(success);
		}

		ref<World> world;
		Entity entity;
		uint8 buffer[1024];
		uint64 size;
	};

	template <typename Value_, typename Action_, typename... Args_>
	void eeditor::undoable(const char* undoLabel, Value_ before, Value_& value,
		Args_... args)
	{
		if (ImGui::IsItemActive() && !ImGui::IsItemActiveLastFrame())
		{
			currentUndoBuffer->as<Value_>() = before;
		}

		bool changed = ImGui::IsItemDeactivatedAfterEdit();
		if constexpr (std::is_enum_v<Value_> || std::is_integral_v<Value_>)
		{
			changed |= (!ImGui::IsItemActive() && (before != value));
		}

		if (changed)
		{
			std::stringstream s;
			s << std::setprecision(4) << std::boolalpha;
			s << undoLabel << " from " << currentUndoBuffer->as<Value_>() << " to " << value;
			Action_ action(value, currentUndoBuffer->as<Value_>(), std::forward<Args_>(args)...);
			currentUndoStack->pushAction(s.str().c_str(), action);
		}
	}

#define UNDOABLE_COMPONENT_SETTING(undoLabel, val, command, ...)					\
	{																				\
		using value_t = std::decay_t<decltype(val)>;								\
		value_t before = val;														\
		command;																	\
		undoable<value_t, component_member_undo<component_t, value_t>>(undoLabel,	\
			before, val, selectedEntity, __VA_ARGS__);								\
	}

#define UNDOABLE_SETTING(undoLabel, val, command)									\
	{																				\
		using value_t = std::decay_t<decltype(val)>;								\
		value_t before = val;														\
		command;																	\
		undoable<value_t, settings_undo<value_t>>(undoLabel,						\
			before, val);															\
	}

	void eeditor::updateSelectedEntityUIRotation()
	{
		if (selectedEntity.is_valid())
		{
			quat rotation = quat::identity;

			if (TransformComponent* transform = selectedEntity.get_component_if_exists<TransformComponent>())
			{
				rotation = transform->get_world_rotation();
			}

			selectedEntityEulerRotation = quat_to_euler(rotation);
		}
	}

	void eeditor::setSelectedEntity(const Entity& entity)
	{
		selectedEntity = entity;
		updateSelectedEntityUIRotation();
	}

	void eeditor::initialize(ref<EditorScene> _scene, main_renderer* _renderer, editor_panels* _editorPanels)
	{
		scene = _scene;
		renderer = _renderer;
		editorPanels = _editorPanels;
		cameraController.initialize(&scene->camera);

		systemInfo = getSystemInfo();
	}

	bool eeditor::update(const UserInput& input, ldr_render_pass* ldrRenderPass, float dt)
	{
		CPU_PROFILE_BLOCK("Update editor");

		currentUndoStack = &undoStacks[this->scene->mode > 0];
		currentUndoBuffer = &undoBuffers[this->scene->mode > 0];

		Entity selectedEntityBefore = selectedEntity;

		ref<World> world = scene->get_current_world();

		// Clear selected entity, if it became invalid (e.g. if it was deleted).
		if (!selectedEntity.is_valid())
		{
			setSelectedEntity({});
		}

		bool objectChanged = false;
		objectChanged |= handleUserInput(input, ldrRenderPass, dt);
		objectChanged |= drawSceneHierarchy();
		objectChanged |= drawMainMenuBar();
		drawSettings(dt);

		if (objectChanged)
		{
			onObjectMoved();
		}

		// This is triggered on undo.
		if (selectedEntity != selectedEntityBefore)
		{
			setSelectedEntity(selectedEntity);
		}

		return objectChanged;
	}

	void eeditor::render(ldr_render_pass* ldrRenderPass, float dt)
	{
		//TODO
	}

	static void drawIconsWindow(bool& open)
	{
		if (open)
		{
			if (ImGui::Begin(ICON_FA_ICONS "  Icons", &open))
			{
				static ImGuiTextFilter filter;
				filter.Draw();

				if (ImGui::BeginChild("Icons List"))
				{
					for (uint32 i = 0; i < arraysize(awesomeIcons); ++i)
					{
						ImGui::PushID(i);
						if (filter.PassFilter(awesomeIconNames[i]))
						{
							ImGui::Text("%s: %s", awesomeIconNames[i], awesomeIcons[i]);
							ImGui::SameLine();
							if (ImGui::Button("Copy to clipboard"))
							{
								ImGui::SetClipboardText(awesomeIconNames[i]);
							}
						}
						ImGui::PopID();
					}
				}
				ImGui::EndChild();
			}
			ImGui::End();
		}
	}

	bool eeditor::drawMainMenuBar()
	{
		static bool iconsWindowOpen = false;
		static bool demoWindowOpen = false;
		static bool undoWindowOpen = false;
		static bool soundEditorWindowOpen = false;

		bool objectPotentiallyMoved = false;

		bool controlsClicked = false;
		bool aboutClicked = false;
		bool systemClicked = false;

		if (ImGui::BeginMainMenuBar())
		{
			if (ImGui::BeginMenu(ICON_FA_FILE "  Project"))
			{
				char textBuffer[128];
				auto [undoPossible, undoName] = currentUndoStack->undoPossible();
				snprintf(textBuffer, sizeof(textBuffer), ICON_FA_UNDO " Undo %s", undoPossible ? undoName : "");
				if (ImGui::MenuItem(textBuffer, "Ctrl+Z", false, undoPossible))
				{
					currentUndoStack->undo();
					objectPotentiallyMoved = true;
				}

				auto [redoPossible, redoName] = currentUndoStack->redoPossible();
				snprintf(textBuffer, sizeof(textBuffer), ICON_FA_REDO " Redo %s", redoPossible ? redoName : "");
				if (ImGui::MenuItem(textBuffer, "Ctrl+Y", false, redoPossible))
				{
					currentUndoStack->redo();
					objectPotentiallyMoved = true;
				}

				if (ImGui::MenuItem(undoWindowOpen ? (ICON_FA_HISTORY "  Hide undo history") : (ICON_FA_HISTORY "  Show undo history")))
				{
					undoWindowOpen = !undoWindowOpen;
				}
				ImGui::Separator();

				if (ImGui::MenuItem(ICON_FA_SAVE "  Save scene", "Ctrl+S"))
				{
					serializeToFile();
				}

				/*if (ImGui::MenuItem(ICON_FA_SAVE "  Build", "Ctrl+B"))
				{
					struct build_data {} bd;
					lowPriorityJobQueue.createJob<build_data>([](build_data& data, JobHandle)
						{
							if (!era_engine::build::ebuilder::build())
								LOG_ERROR("Building> Failed to build the game.");
						}, bd).submitNow();
				}*/

				if (ImGui::MenuItem(ICON_FA_FOLDER_OPEN "  Load scene", "Ctrl+O"))
				{
					forceStop();
					deserializeFromFile();
				}

				ImGui::Separator();
				if (ImGui::MenuItem(ICON_FA_TIMES "  Exit", "Esc"))
				{
					PostQuitMessage(0);
				}
				ImGui::EndMenu();
			}

			if (ImGui::BeginMenu(ICON_FA_TOOLS "  Tools"))
			{
				if (ImGui::MenuItem(iconsWindowOpen ? (ICON_FA_ICONS "  Hide available icons") : (ICON_FA_ICONS "  Show available icons")))
				{
					iconsWindowOpen = !iconsWindowOpen;
				}

				if (ImGui::MenuItem(demoWindowOpen ? (ICON_FA_PUZZLE_PIECE "  Hide demo window") : (ICON_FA_PUZZLE_PIECE "  Show demo window")))
				{
					demoWindowOpen = !demoWindowOpen;
				}

				ImGui::Separator();

				if (ImGui::MenuItem(dxProfilerWindowOpen ? (ICON_FA_CHART_BAR "  Hide GPU profiler") : (ICON_FA_CHART_BAR "  Show GPU profiler"), nullptr, nullptr, ENABLE_DX_PROFILING))
				{
					dxProfilerWindowOpen = !dxProfilerWindowOpen;
				}

				if (ImGui::MenuItem(cpu_profiler_window_open ? (ICON_FA_CHART_LINE "  Hide CPU profiler") : (ICON_FA_CHART_LINE "  Show CPU profiler"), nullptr, nullptr, ENABLE_CPU_PROFILING))
				{
					cpu_profiler_window_open = !cpu_profiler_window_open;
				}

				ImGui::Separator();

#ifdef ENABLE_MESSAGE_LOG
				if (ImGui::MenuItem(log_window_open ? (ICON_FA_CLIPBOARD_LIST "  Hide Console") : (ICON_FA_CLIPBOARD_LIST "  Show Console"), "Ctrl+L", nullptr, 1))
#else
				if (ImGui::MenuItem(log_window_open ? (ICON_FA_CLIPBOARD_LIST "  Hide Console") : (ICON_FA_CLIPBOARD_LIST "  Show Console"), "Ctrl+L", nullptr, 0))
#endif
				{
					log_window_open = !log_window_open;
				}

				ImGui::Separator();

				if (ImGui::MenuItem(soundEditorWindowOpen ? (EDITOR_ICON_AUDIO "  Hide sound editor") : (EDITOR_ICON_AUDIO "  Show sound editor")))
				{
					soundEditorWindowOpen = !soundEditorWindowOpen;
				}

				if (ImGui::MenuItem(editorPanels->meshEditor.isOpen() ? (EDITOR_ICON_MESH "  Hide mesh editor") : (EDITOR_ICON_MESH "  Show mesh editor")))
				{
					if (editorPanels->meshEditor.isOpen())
						editorPanels->meshEditor.close();
					else
					{
						editorPanels->meshEditor.open();
						editorPanels->meshEditor.setScene(scene);
					}
				}

				ImGui::Separator();

				if (ImGui::MenuItem(ICON_FA_DESKTOP "  System"))
				{
					systemClicked = true;
				}

				ImGui::EndMenu();
			}

			if (ImGui::BeginMenu(ICON_FA_QUESTION "  Help"))
			{
				// TODO: Docs
				if (ImGui::MenuItem(ICON_FA_MEMORY "  Repository"))
					os::system_calls::openURL("https://github.com/EldarMuradov/EraEngine");

				ImGui::EndMenu();
			}

			if (ImGui::BeginMenu(ICON_FA_QUESTION "  About"))
			{
				if (ImGui::MenuItem(ICON_FA_QUESTION "  Author"))
				{
					aboutClicked = true;
				}

				ImGui::EndMenu();
			}

			ImGui::EndMainMenuBar();
		}

		ImVec2 center = ImGui::GetMainViewport()->GetCenter();

		if (systemClicked)
		{
			ImGui::OpenPopup(ICON_FA_DESKTOP "  System");
		}

		ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
		if (ImGui::BeginPopupModal(ICON_FA_DESKTOP "  System", 0, ImGuiWindowFlags_AlwaysAutoResize))
		{
			ImGui::Value("CPU", systemInfo.cpuName.c_str());
			ImGui::Value("GPU", systemInfo.gpuName.c_str());

			ImGui::Separator();

			ImGui::PopupOkButton();
			ImGui::EndPopup();
		}

		if (aboutClicked)
		{
			ImGui::OpenPopup(ICON_FA_QUESTION "  About");
		}

		ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
		if (ImGui::BeginPopupModal(ICON_FA_QUESTION "  About", 0, ImGuiWindowFlags_AlwaysAutoResize))
		{
			ImGui::Dummy(ImVec2(300.f, 1.f));
			ImGui::CenteredText("Era Engine");
			ImGui::CenteredText("by Eldar Muradov");

			ImGui::Separator();

			ImGui::PopupOkButton();
			ImGui::EndPopup();
		}

		if (demoWindowOpen)
		{
			ImGui::ShowDemoWindow(&demoWindowOpen);
		}

		drawIconsWindow(iconsWindowOpen);
		drawSoundEditor(soundEditorWindowOpen);
		objectPotentiallyMoved |= currentUndoStack->showHistory(undoWindowOpen);

		return objectPotentiallyMoved;
	}

	static bounding_box getObjectBoundingBox(Entity entity, bool applyPosition)
	{
		bounding_box aabb = entity.has_component<MeshComponent>() ? entity.get_component<MeshComponent>()->mesh->aabb : bounding_box::fromCenterRadius(0.f, 1.f);

		if (TransformComponent* transform = entity.get_component_if_exists<TransformComponent>())
		{
			const trs& world_transform = transform->get_world_transform();

			aabb.minCorner *= world_transform.scale;
			aabb.maxCorner *= world_transform.scale;

			if (applyPosition)
			{
				aabb.minCorner += world_transform.position;
				aabb.maxCorner += world_transform.position;
			}
		}
		
		return aabb;
	}

	void eeditor::renderChilds(Entity& entity)
	{
		static int i = 0;

		ref<World> world = scene->get_current_world();

		auto childs = EntityContainer::get_childs(world.get(), entity.get_handle());

		if (!childs.empty())
		{
			auto iter = childs.begin();
			const auto& end = childs.end();
			for (; iter != end; ++iter)
			{
				Entity child = world->get_entity(*iter);
				NameComponent* name_component = child.get_component_if_exists<NameComponent>();
				ImGuiTreeNodeFlags flags = ((selectedEntity == *iter) ? ImGuiTreeNodeFlags_Selected : 0) | ImGuiTreeNodeFlags_OpenOnArrow;
				if (name_component)
				{
					if (ImGui::TreeNodeEx((void*)(uint32)(*iter), flags, name_component->name))
					{
						if (ImGui::IsItemClicked())
						{
							setSelectedEntity(child);
						}

						renderChilds(child);

						ImGui::TreePop();
					}
				}
				else
				{
					if (ImGui::TreeNodeEx((void*)(uint32)(*iter), flags, std::to_string(i++).c_str()))
					{
						if (ImGui::IsItemClicked())
						{
							setSelectedEntity(child);
						}

						renderChilds(child);

						ImGui::TreePop();
					}
				}
			}
		}
	}

	static void draw_float_wrap(float wdt, const char* label, float* v, float v_speed = 1.0f, float v_min = 0.0f, float v_max = 0.0f, const char* format = "%.3f", const ImGuiSliderFlags flags = 0)
	{
		static const uint32_t screen_edge_padding = 10;
		static bool needs_to_wrap = false;
		ImGuiIO& imgui_io = ImGui::GetIO();

		if (ImGui::IsMouseDragging(ImGuiMouseButton_Left))
		{
			ImVec2 position_cursor = imgui_io.MousePos;
			float position_left = static_cast<float>(screen_edge_padding);
			float position_right = static_cast<float>(wdt - screen_edge_padding);
			bool is_on_right_screen_edge = position_cursor.x >= position_right;
			bool is_on_left_screen_edge = position_cursor.x <= position_left;

			if (is_on_right_screen_edge)
			{
				position_cursor.x = position_left + 2;
				needs_to_wrap = true;
			}
			else if (is_on_left_screen_edge)
			{
				position_cursor.x = position_right - 2;
				needs_to_wrap = true;
			}
		}

		ImGui::PushID(static_cast<int>(ImGui::GetCursorPosX() + ImGui::GetCursorPosY()));
		ImGui::DragFloat(label, v, v_speed, v_min, v_max, format, flags);
		ImGui::PopID();
	}

	static void showVector3(const char* label, vec3& vector, float wdt)
	{
		const float label_indetation = 15.0f;

		const auto show_float = [wdt](vec3 axis, float* value)
			{
				const float label_float_spacing = 15.0f;
				const float step = 0.01f;

				// Label
				ImGui::TextUnformatted(axis.x == 1.0f ? "x" : axis.y == 1.0f ? "y" : "z");
				ImGui::SameLine(label_float_spacing);
				ImVec2 cp = ImGui::GetCursorScreenPos();
				vec2 pos_post_label = vec2(cp.x, cp.y);

				// Float
				ImGui::PushItemWidth(128.0f);
				ImGui::PushID(static_cast<int>(ImGui::GetCursorPosX() + ImGui::GetCursorPosY()));
				draw_float_wrap(wdt, "##no_label", value, step, std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max(), "%.4f");
				ImGui::PopID();
				ImGui::PopItemWidth();

				// Axis color
				static const ImU32 color_x = IM_COL32(168, 46, 2, 255);
				static const ImU32 color_y = IM_COL32(112, 162, 22, 255);
				static const ImU32 color_z = IM_COL32(51, 122, 210, 255);
				static const vec2 size = vec2(4.0f, 19.0f);
				static const vec2 offset = vec2(5.0f, 4.0);
				pos_post_label += offset;
				ImRect axis_color_rect = ImRect(pos_post_label.x, pos_post_label.y, pos_post_label.x + size.x, pos_post_label.y + size.y);
				ImGui::GetWindowDrawList()->AddRectFilled(axis_color_rect.Min, axis_color_rect.Max, axis.x == 1.0f ? color_x : axis.y == 1.0f ? color_y : color_z);
			};

		ImGui::BeginGroup();
		ImGui::Indent(label_indetation);
		ImGui::TextUnformatted(label);
		ImGui::Unindent(label_indetation);
		show_float(vec3(1.0f, 0.0f, 0.0f), &vector.x);
		show_float(vec3(0.0f, 1.0f, 0.0f), &vector.y);
		show_float(vec3(0.0f, 0.0f, 1.0f), &vector.z);
		ImGui::EndGroup();
	};

	bool eeditor::drawSceneHierarchy()
	{
		ref<World> world = scene->get_current_world();

		bool objectMovedByWidget = false;

		if (ImGui::Begin("Scene Hierarchy"))
		{
			if (ImGui::BeginChild("Outliner"))
			{
				world->view<NameComponent>()
					.each([this, &world](Entity::Handle entityHandle, NameComponent& tag)
						{
							ImGui::PushID((uint32)entityHandle);
							const char* name = tag.name;
							Entity entity = world->get_entity(entityHandle);

							Entity parent = world->get_entity(entity.get_parent_handle());

							if (parent.is_valid())
							{
								ImGuiTreeNodeFlags flags = ((selectedEntity == entity) ? ImGuiTreeNodeFlags_Selected : 0) | ImGuiTreeNodeFlags_OpenOnArrow;
								if (ImGui::TreeNodeEx((void*)(uint32)entityHandle, flags, name))
								{
									if (ImGui::IsItemClicked())
									{
										setSelectedEntity(entity);
									}

									renderChilds(entity);

									ImGui::TreePop();
								}
							}

							bool entityDeleted = false;
							if (ImGui::BeginPopupContextItem(name))
							{
								if (ImGui::MenuItem("Delete"))
								{
									entityDeleted = true;
								}

								ImGui::EndPopup();
							}

							if (entityDeleted)
							{
								if (entity == selectedEntity)
								{
									setSelectedEntity({});
								}
								//currentUndoStack->pushAction("entity deletion", entity_existence_undo(world, entity)); //TODO
								world->destroy_entity(entity);
							}
							ImGui::PopID();
						});
			}
			ImGui::EndChild();

			if (selectedEntity.is_valid())
			{
				if (ImGui::Begin("Components"))
				{
					ImGui::AlignTextToFramePadding();

					ImGui::PushID((uint32)selectedEntity.get_handle());

					{
						NameComponent* name_component = selectedEntity.get_component_if_exists<NameComponent>();

						NameComponent tagBefore = *name_component;
						ImGui::InputText("Name", name_component->name, sizeof(NameComponent::name));

						if (ImGui::IsItemActive() && !ImGui::IsItemActiveLastFrame())
						{
							currentUndoBuffer->as<NameComponent>() = tagBefore;
						}
						if (ImGui::IsItemDeactivatedAfterEdit())
						{
							//TODO
							//currentUndoStack->pushAction("entity name", component_undo<NameComponent>(selectedEntity, currentUndoBuffer->as<NameComponent>()));
						}
					}

					ImGui::PopID();

					ImGui::SameLine();
					if (ImGui::Button(ICON_FA_TRASH_ALT))
					{
						//currentUndoStack->pushAction("entity deletion", entity_existence_undo(world, selectedEntity));
						world->destroy_entity(selectedEntity);
						setSelectedEntity({});
						objectMovedByWidget = true;
					}

					if (ImGui::IsItemHovered())
					{
						ImGui::SetTooltip("Delete entity");
					}

					if (selectedEntity.is_valid())
					{
						EntityEditorUtils::edit_entity(world.get(), selectedEntity.get_handle());
					}

					if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right) && !ImGui::IsAnyItemHovered())
					{
						ImGui::OpenPopup("CreateComponentPopup");
					}

					if (ImGui::BeginPopup("CreateComponentPopup"))
					{
						ImGui::Text("Create component");
						ImGui::Separator();

						//for (const auto& script : era_engine::dotnet::enative_scripting_linker::scriptTypes)
						//{
						//	if (ImGui::MenuItem(script.c_str()))
						//	{
						//		era_engine::dotnet::enative_scripting_linker::createScript((int)selectedEntity.get_handle(), script.c_str());
						//	}
						//}

						ImGui::EndPopup();
					}
				}
				ImGui::End();
			}
		}
		ImGui::End();

		return objectMovedByWidget;
	}

	void eeditor::onObjectMoved()
	{
		if (selectedEntity.is_valid())
		{
			//if (physics::px_cloth_component* cloth = selectedEntity.get_component_if_exists<physics::px_cloth_component>())
			//{
			//	cloth->translate(selectedEntity.get_component<TransformComponent>().transform.position);
			//}
		}
	}

	bool eeditor::handleUserInput(const UserInput& input, ldr_render_pass* ldrRenderPass, float dt)
	{
		ref<World> world = this->scene->get_current_world();

		// Returns true, if the user dragged an object using a gizmo.

		if (input.keyboard['F'].press_event && selectedEntity.is_valid())
		{
			bounding_box aabb = getObjectBoundingBox(selectedEntity, true);
			cameraController.centerCameraOnObject(aabb);
		}

		bool inputCaptured = cameraController.update(input, renderer->renderWidth, renderer->renderHeight, dt);

		if (inputCaptured)
		{
			renderer->pathTracer.resetRendering();
		}

		bool objectMovedByGizmo = false;

		render_camera& camera = this->scene->is_pausable() ? this->scene->editor_camera : this->scene->camera;

		if (&camera != cameraController.camera)
		{
			cameraController.camera = &camera;
		}

		bool gizmoDrawn = false;

		if (!inputCaptured && !ImGui::IsAnyItemActive() && ImGui::IsKeyDown(key_shift) && ImGui::IsKeyPressed('A'))
		{
			ImGui::OpenPopup("CreateEntityPopup");
			inputCaptured = true;
		}

		inputCaptured |= drawEntityCreationPopup();

		if (ImGui::BeginControlsWindow("##EntityControls", ImVec2(0.f, 0.f), ImVec2(20.f, 90.f)))
		{
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));

			if (ImGui::Button(ICON_FA_PLUS)) { ImGui::OpenPopup("CreateEntityPopup"); }
			if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Create entity (Shift+A)"); }
			inputCaptured |= drawEntityCreationPopup();

			ImGui::PopStyleColor();
		}
		ImGui::End();

		bool draggingBefore = gizmo.dragging;

		if (selectedEntity.is_valid())
		{
			if (TransformComponent* transform = selectedEntity.get_component_if_exists<TransformComponent>())
			{
				//if (gizmo.manipulateTransformation(transform->get_world_transform(), camera, input, !inputCaptured, ldrRenderPass))
				{
					/*if (auto rb = selectedEntity.get_component_if_exists<physics::DynamicBodyComponent>())
					{
						rb->manual_set_physics_position_and_rotation(transform->transform.position, transform->transform.rotation);
					}
					else if (auto rb = selectedEntity.get_component_if_exists<physics::StaticBodyComponent>())
					{
						rb->manual_set_physics_position_and_rotation(transform->transform.position, transform->transform.rotation);
					}*/

					//if (auto vehicle = physics::getVehicleComponent(selectedEntity))
					//{
					//	vehicle->setTransform(transform->position, transform->rotation);
					//}

					//if (physics::px_cloth_component* cloth = selectedEntity.getComponentIfExists<physics::px_cloth_component>())
					//	cloth->translate(selectedEntity.getComponent<transform_component>().position);

					updateSelectedEntityUIRotation();
					inputCaptured = true;
					objectMovedByGizmo = true;
				}
				//else if (draggingBefore)
				{
					//currentUndoStack->pushAction("object transform",
					//	component_undo<TransformComponent>(selectedEntity, TransformComponent(selectedEntity.internal_data, gizmo.originalTransform)));
				}
			}
			else
			{
				gizmo.manipulateNothing(camera, input, !inputCaptured, ldrRenderPass);
			}

			if (!inputCaptured && !ImGui::IsAnyItemActive())
			{
				if (ImGui::IsKeyPressed(key_backspace) || ImGui::IsKeyPressed(key_delete))
				{
					// Delete entity.
					//currentUndoStack->pushAction("entity deletion", entity_existence_undo(world, selectedEntity));
					world->destroy_entity(selectedEntity);
					setSelectedEntity({});
					inputCaptured = true;
					objectMovedByGizmo = true;
				}
				else if (ImGui::IsKeyDown(key_ctrl) && ImGui::IsKeyPressed('D'))
				{
					// Duplicate entity.
					//Entity newEntity = world->copyEntity(selectedEntity);
					//setSelectedEntity(newEntity);
					//TODO
					//inputCaptured = true;
					//objectMovedByGizmo = true;
				}
			}
		}
		else
		{
			gizmo.manipulateNothing(camera, input, !inputCaptured, ldrRenderPass);
		}

		float simControlsWidth = IMGUI_ICON_DEFAULT_SIZE * 3 + IMGUI_ICON_DEFAULT_SPACING * 2 + ImGui::GetStyle().WindowPadding.x * 2;

		if (ImGui::BeginControlsWindow("##SimulationControls", ImVec2(0.5f, 0.f), ImVec2(-simControlsWidth * 0.5f, 20.f)))
		{
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));

			//if (ImGui::IconButton(imgui_icon_play, imgui_icon_play, IMGUI_ICON_DEFAULT_SIZE, this->scene->is_playable()))
			//{
			//	setSelectedEntity({});
			//	forceStart();
			//}
			//ImGui::SameLine(0.f, IMGUI_ICON_DEFAULT_SPACING);
			if (ImGui::IconButton(imgui_icon_pause, imgui_icon_pause, IMGUI_ICON_DEFAULT_SIZE, this->scene->is_pausable()))
			{
				forcePause();
			}
			//ImGui::SameLine(0.f, IMGUI_ICON_DEFAULT_SPACING);
			//if (ImGui::IconButton(imgui_icon_stop, imgui_icon_stop, IMGUI_ICON_DEFAULT_SIZE, this->scene->is_stoppable()))
			//{
			//	forceStop();
			//}

			//scene = &this->scene->getCurrentScene();
			//cameraController.camera = &camera;

			currentUndoStack = &undoStacks[this->scene->mode > 0];
			currentUndoBuffer = &undoBuffers[this->scene->mode > 0];

			ImGui::PopStyleColor();
		}
		ImGui::End();

		if (!ImGui::IsAnyItemActive())
		{
			if (!inputCaptured && ImGui::IsKeyDown(key_ctrl) && ImGui::IsKeyPressed('Z'))
			{
				currentUndoStack->undo();
				inputCaptured = true;
				objectMovedByGizmo = true;
			}
			if (!inputCaptured && ImGui::IsKeyDown(key_ctrl) && ImGui::IsKeyPressed('Y'))
			{
				currentUndoStack->redo();
				inputCaptured = true;
				objectMovedByGizmo = true;
			}
			if (!inputCaptured && ImGui::IsKeyDown(key_ctrl) && ImGui::IsKeyPressed('S'))
			{
				serializeToFile();
				inputCaptured = true;
				ImGui::GetIO().KeysDown['S'] = false; // Hack: Window does not get notified of inputs due to the file dialog.
			}
			if (!inputCaptured && ImGui::IsKeyDown(key_ctrl) && ImGui::IsKeyPressed('O'))
			{
				deserializeFromFile();
				inputCaptured = true;
				ImGui::GetIO().KeysDown['O'] = false; // Hack: Window does not get notified of inputs due to the file dialog.
			}
			if (!inputCaptured && ImGui::IsKeyDown(key_ctrl) && ImGui::IsKeyPressed('L'))
			{
				log_window_open = !log_window_open;
				inputCaptured = true;
			}
		}

		if (!inputCaptured && input.mouse.left.click_event)
		{
			if (input.keyboard[key_ctrl].down)
			{
				vec3 dir = -camera.generateWorldSpaceRay(input.mouse.relX, input.mouse.relY).direction;
				vec3 beforeDir = this->scene->sun.direction;
				this->scene->sun.direction = dir;
				//currentUndoStack->pushAction("sun direction", settings_undo<vec3>(this->scene->sun.direction, beforeDir));
				inputCaptured = true;
			}
			else
			{
				if (renderer->hoveredObjectID != -1)
				{
					setSelectedEntity(world->get_entity(Entity::Handle{ renderer->hoveredObjectID }));
				}
				else
				{
					setSelectedEntity({});
				}
			}
			inputCaptured = true;
		}

		return objectMovedByGizmo;
	}

	bool eeditor::drawEntityCreationPopup()
	{
		ref<World> world = scene->get_current_world();
		render_camera& camera = this->scene->camera;

		bool clicked = false;

		if (ImGui::BeginPopup("CreateEntityPopup"))
		{
			if (ImGui::MenuItem("Point light", "P") || ImGui::IsKeyPressed('P'))
			{
				//Entity pl = world->create_entity("Point light")
				//	.add_component<PointLightComponent>(
				//		vec3(1.f, 1.f, 1.f),
				//		1.f,
				//		10.f,
				//		true,
				//		2048u
				//	);

				//TransformComponent& transform = pl.get_component<TransformComponent>();
				//transform.transform.position = camera.position + camera.rotation * vec3(0.f, 0.f, -3.f);

				//currentUndoStack->pushAction("entity creation", entity_existence_undo(world, pl));

				//setSelectedEntity(pl);
				clicked = true;
			}

			if (ImGui::MenuItem("Spot light", "S") || ImGui::IsKeyPressed('S'))
			{
				//Entity sl = world->create_entity("Spot light")
				//	.add_component<SpotLightComponent>(
				//		vec3(1.f, 1.f, 1.f),
				//		1.f,
				//		25.f,
				//		deg2rad(20.f),
				//		deg2rad(30.f),
				//		true,
				//		2048u
				//	);

				//TransformComponent& transform = sl.get_component<TransformComponent>();
				//transform.transform.position = camera.position + camera.rotation * vec3(0.f, 0.f, -3.f);

				//currentUndoStack->pushAction("entity creation", entity_existence_undo(world, sl));

				//setSelectedEntity(sl);
				clicked = true;
			}

			ImGui::Separator();

			if (ImGui::MenuItem("Empty", "E") || ImGui::IsKeyPressed('E'))
			{
				auto empty = world->create_entity("Empty");
				TransformComponent* transform = empty.get_component<TransformComponent>();
				transform->set_world_position(camera.position + camera.rotation * vec3(0.f, 0.f, -3.f));

				//currentUndoStack->pushAction("entity creation", entity_existence_undo(world, empty));

				setSelectedEntity(empty);
				clicked = true;
			}

			if (clicked)
			{
				ImGui::CloseCurrentPopup();
			}

			ImGui::EndPopup();
		}

		return clicked;
	}

	void eeditor::serializeToFile()
	{
		//serializeSceneToYAMLFile(*scene, renderer->settings);
	}

	bool eeditor::deserializeFromFile()
	{
		////std::string environmentName;
		////if (deserializeSceneFromYAMLFile(*scene, renderer->settings, environmentName))
		////{
		////	scene->stop();

		////	setSelectedEntity({});
		////	scene->environment.setFromTexture(environmentName);
		////	scene->environment.forceUpdate(this->scene->sun.direction);
		////	renderer->pathTracer.resetRendering();

		////	return true;
		////}
		return false;
	}

	bool eeditor::deserializeFromCurrentFile(const fs::path& path)
	{
		//std::string environmentName;
		//if (deserializeSceneFromCurrentYAMLFile(path, *scene, renderer->settings, environmentName))
		//{
		//	scene->stop();

		//	setSelectedEntity({});
		//	scene->environment.setFromTexture(environmentName);
		//	scene->environment.forceUpdate(this->scene->sun.direction);
		//	renderer->pathTracer.resetRendering();

		//	return true;
		//}
		return false;
	}

	bool eeditor::editCamera(render_camera& camera)
	{
		bool result = false;
		if (ImGui::BeginTree("Camera"))
		{
			UNDOABLE_SETTING("position", camera.position,
				result |= ImGui::DragFloat3("Position", camera.position.data));

			if (ImGui::BeginProperties())
			{
				UNDOABLE_SETTING("field of view", camera.verticalFOV,
					result |= ImGui::PropertySliderAngle("Field of view", camera.verticalFOV, 1.f, 150.f));
				UNDOABLE_SETTING("near plane", camera.nearPlane,
					result |= ImGui::PropertyDrag("Near plane", camera.nearPlane, 0.01f, 0.f));

				bool infiniteFarplane = camera.farPlane < 0.f;
				UNDOABLE_SETTING("infinite far plane", camera.farPlane,
					if (ImGui::PropertyCheckbox("Infinite far plane", infiniteFarplane))
					{
						if (!infiniteFarplane)
						{
							camera.farPlane = (camera.farPlane == -1.f) ? 500.f : -camera.farPlane;
						}
						else
						{
							camera.farPlane = -camera.farPlane;
						}
						result = true;
					});
				if (!infiniteFarplane)
				{
					UNDOABLE_SETTING("far plane", camera.farPlane,
						result |= ImGui::PropertyDrag("Far plane", camera.farPlane, 0.1f, 0.f));
				}

				ImGui::EndProperties();
			}

			ImGui::EndTree();
		}
		return result;
	}

	bool eeditor::editTonemapping(tonemap_settings& tonemap)
	{
		bool result = false;
		if (ImGui::BeginTree("Tonemapping"))
		{
			ImGui::PlotLines("",
				[](void* data, int idx)
				{
					float t = idx * 0.01f;
					tonemap_settings& aces = *(tonemap_settings*)data;
					return aces.tonemap(t);
				},
				&tonemap, 100, 0, 0, 0.f, 1.f, ImVec2(250.f, 250.f));

			if (ImGui::BeginProperties())
			{
				UNDOABLE_SETTING("shoulder strength", tonemap.A,
					result |= ImGui::PropertySlider("Shoulder strength", tonemap.A, 0.f, 1.f));
				UNDOABLE_SETTING("linear strength", tonemap.B,
					result |= ImGui::PropertySlider("Linear strength", tonemap.B, 0.f, 1.f));
				UNDOABLE_SETTING("linear angle", tonemap.C,
					result |= ImGui::PropertySlider("Linear angle", tonemap.C, 0.f, 1.f));
				UNDOABLE_SETTING("toe strength", tonemap.D,
					result |= ImGui::PropertySlider("Toe strength", tonemap.D, 0.f, 1.f));
				UNDOABLE_SETTING("tone numerator", tonemap.E,
					result |= ImGui::PropertySlider("Tone numerator", tonemap.E, 0.f, 1.f));
				UNDOABLE_SETTING("toe denominator", tonemap.F,
					result |= ImGui::PropertySlider("Toe denominator", tonemap.F, 0.f, 1.f));
				UNDOABLE_SETTING("linear white", tonemap.linearWhite,
					result |= ImGui::PropertySlider("Linear white", tonemap.linearWhite, 0.f, 100.f));
				UNDOABLE_SETTING("exposure", tonemap.exposure,
					result |= ImGui::PropertySlider("Exposure", tonemap.exposure, -3.f, 3.f));
				ImGui::EndProperties();
			}

			ImGui::EndTree();
		}
		return result;
	}

	bool eeditor::editSunShadowParameters(directional_light& sun)
	{
		bool result = false;
		if (ImGui::BeginTree("Sun"))
		{
			if (ImGui::BeginProperties())
			{
				UNDOABLE_SETTING("sun intensity", sun.intensity,
					result |= ImGui::PropertySlider("Intensity", sun.intensity, 0.f, 1000.f));
				UNDOABLE_SETTING("sun color", sun.color,
					result |= ImGui::PropertyColorWheel("Color", sun.color));

				UNDOABLE_SETTING("sun shadow resolution", sun.shadowDimensions,
					result |= ImGui::PropertyDropdownPowerOfTwo("Shadow resolution", 128, 2048, sun.shadowDimensions));
				UNDOABLE_SETTING("stabilize", sun.stabilize,
					result |= ImGui::PropertyCheckbox("Stabilize", sun.stabilize));
				UNDOABLE_SETTING("negative z-offset", sun.negativeZOffset,
					result |= ImGui::PropertyDrag("Negative z-offset", sun.negativeZOffset, 0.5f, 0.f));

				UNDOABLE_SETTING("cascade count", sun.numShadowCascades,
					result |= ImGui::PropertySlider("Cascade count", sun.numShadowCascades, 1, 4));

				const float minCascadeDistance = 0.f, maxCascadeDistance = 300.f;
				const float minBias = 0.f, maxBias = 0.0015f;
				const float minBlend = 0.f, maxBlend = 10.f;
				if (sun.numShadowCascades == 1)
				{
					UNDOABLE_SETTING("cascade distance", sun.cascadeDistances.x,
						result |= ImGui::PropertySlider("Distance", sun.cascadeDistances.x, minCascadeDistance, maxCascadeDistance));
					UNDOABLE_SETTING("cascade bias", sun.bias.x,
						result |= ImGui::PropertySlider("Bias", sun.bias.x, minBias, maxBias, "%.6f"));
					UNDOABLE_SETTING("cascade blend distances", sun.blendDistances.x,
						result |= ImGui::PropertySlider("Blend distances", sun.blendDistances.x, minBlend, maxBlend, "%.6f"));
				}
				else if (sun.numShadowCascades == 2)
				{
					UNDOABLE_SETTING("cascade distance", sun.cascadeDistances.xy,
						result |= ImGui::PropertySlider("Distance", sun.cascadeDistances.xy, minCascadeDistance, maxCascadeDistance));
					UNDOABLE_SETTING("cascade bias", sun.bias.xy,
						result |= ImGui::PropertySlider("Bias", sun.bias.xy, minBias, maxBias, "%.6f"));
					UNDOABLE_SETTING("cascade blend distances", sun.blendDistances.xy,
						result |= ImGui::PropertySlider("Blend distances", sun.blendDistances.xy, minBlend, maxBlend, "%.6f"));
				}
				else if (sun.numShadowCascades == 3)
				{
					UNDOABLE_SETTING("cascade distance", sun.cascadeDistances.xyz,
						result |= ImGui::PropertySlider("Distance", sun.cascadeDistances.xyz, minCascadeDistance, maxCascadeDistance));
					UNDOABLE_SETTING("cascade bias", sun.bias.xyz,
						result |= ImGui::PropertySlider("Bias", sun.bias.xyz, minBias, maxBias, "%.6f"));
					UNDOABLE_SETTING("cascade blend distances", sun.blendDistances.xyz,
						result |= ImGui::PropertySlider("Blend distances", sun.blendDistances.xyz, minBlend, maxBlend, "%.6f"));
				}
				else if (sun.numShadowCascades == 4)
				{
					UNDOABLE_SETTING("cascade distance", sun.cascadeDistances,
						result |= ImGui::PropertySlider("Distance", sun.cascadeDistances, minCascadeDistance, maxCascadeDistance));
					UNDOABLE_SETTING("cascade bias", sun.bias,
						result |= ImGui::PropertySlider("Bias", sun.bias, minBias, maxBias, "%.6f"));
					UNDOABLE_SETTING("cascade blend distances", sun.blendDistances,
						result |= ImGui::PropertySlider("Blend distances", sun.blendDistances, minBlend, maxBlend, "%.6f"));
				}

				ImGui::EndProperties();
			}

			if (ImGui::BeginTree("Show##ShowShadowMaps"))
			{
				for (uint32 i = 0; i < sun.numShadowCascades; ++i)
				{
					auto vp = sun.shadowMapViewports[i];
					ImGui::Image(render_resources::shadowMap, 0, 0, ImVec2(vp.x, vp.y), ImVec2(vp.x + vp.z, vp.y + vp.w));
				}
				ImGui::EndTree();
			}

			ImGui::EndTree();
		}
		return result;
	}

	bool eeditor::editAO(bool& enable, hbao_settings& settings, const ref<dx_texture>& aoTexture)
	{
		bool result = false;
		if (ImGui::BeginProperties())
		{
			UNDOABLE_SETTING("enable HBAO", enable,
				result |= ImGui::PropertyCheckbox("Enable HBAO", enable));
			if (enable)
			{
				UNDOABLE_SETTING("num AO rays", settings.numRays,
					result |= ImGui::PropertySlider("Num rays", settings.numRays, 1, 16));
				UNDOABLE_SETTING("max num steps per AO ray", settings.maxNumStepsPerRay,
					result |= ImGui::PropertySlider("Max num steps per ray", settings.maxNumStepsPerRay, 1, 16));
				UNDOABLE_SETTING("AO radius", settings.radius,
					result |= ImGui::PropertySlider("Radius", settings.radius, 0.f, 1.f, "%.3fm"));
				UNDOABLE_SETTING("AO strength", settings.strength,
					result |= ImGui::PropertySlider("Strength", settings.strength, 0.f, 2.f));
			}
			ImGui::EndProperties();
		}
		if (enable && aoTexture && ImGui::BeginTree("Show##ShowAO"))
		{
			ImGui::Image(aoTexture);
			ImGui::EndTree();
		}
		return result;
	}

	bool eeditor::editSSS(bool& enable, sss_settings& settings, const ref<dx_texture>& sssTexture)
	{
		bool result = false;
		if (ImGui::BeginProperties())
		{
			UNDOABLE_SETTING("enable SSS", enable,
				result |= ImGui::PropertyCheckbox("Enable SSS", enable));
			if (enable)
			{
				UNDOABLE_SETTING("num SSS iterations", settings.numSteps,
					result |= ImGui::PropertySlider("Num iterations", settings.numSteps, 1, 64));
				UNDOABLE_SETTING("SSS ray distance", settings.rayDistance,
					result |= ImGui::PropertySlider("Ray distance", settings.rayDistance, 0.05f, 3.f, "%.3fm"));
				UNDOABLE_SETTING("SSS thickness", settings.thickness,
					result |= ImGui::PropertySlider("Thickness", settings.thickness, 0.05f, 1.f, "%.3fm"));
				UNDOABLE_SETTING("SSS max distance from camera", settings.maxDistanceFromCamera,
					result |= ImGui::PropertySlider("Max distance from camera", settings.maxDistanceFromCamera, 5.f, 1000.f, "%.3fm"));
				UNDOABLE_SETTING("SSS distance fadeout range", settings.distanceFadeoutRange,
					result |= ImGui::PropertySlider("Distance fadeout range", settings.distanceFadeoutRange, 1.f, 5.f, "%.3fm"));
				UNDOABLE_SETTING("SSS border fadeout", settings.borderFadeout,
					result |= ImGui::PropertySlider("Border fadeout", settings.borderFadeout, 0.f, 0.5f));
			}
			ImGui::EndProperties();
		}
		if (enable && sssTexture && ImGui::BeginTree("Show##ShowSSS"))
		{
			ImGui::Image(sssTexture);
			ImGui::EndTree();
		}
		return result;
	}

	bool eeditor::editSSR(bool& enable, ssr_settings& settings, const ref<dx_texture>& ssrTexture)
	{
		bool result = false;
		if (ImGui::BeginProperties())
		{
			UNDOABLE_SETTING("enable SSR", enable,
				result |= ImGui::PropertyCheckbox("Enable SSR", enable));
			if (enable)
			{
				UNDOABLE_SETTING("num SSR iterations", settings.numSteps,
					result |= ImGui::PropertySlider("Num iterations", settings.numSteps, 1, 1024));
				UNDOABLE_SETTING("SSR max distance", settings.maxDistance,
					result |= ImGui::PropertySlider("Max distance", settings.maxDistance, 5.f, 1000.f, "%.3fm"));
				UNDOABLE_SETTING("SSR min stride", settings.minStride,
					result |= ImGui::PropertySlider("Min stride", settings.minStride, 1.f, 50.f, "%.3fm"));
				UNDOABLE_SETTING("SSR max stride", settings.maxStride,
					result |= ImGui::PropertySlider("Max stride", settings.maxStride, settings.minStride, 50.f, "%.3fm"));
			}
			ImGui::EndProperties();
		}
		if (enable && ssrTexture && ImGui::BeginTree("Show##ShowSSR"))
		{
			ImGui::Image(ssrTexture);
			ImGui::EndTree();
		}
		return result;
	}

	bool eeditor::editTAA(bool& enable, taa_settings& settings, const ref<dx_texture>& velocityTexture)
	{
		bool result = false;
		if (ImGui::BeginProperties())
		{
			UNDOABLE_SETTING("enable TAA", enable,
				result |= ImGui::PropertyCheckbox("Enable TAA", enable));
			if (enable)
			{
				UNDOABLE_SETTING("TAA jitter strength", settings.cameraJitterStrength,
					result |= ImGui::PropertySlider("Jitter strength", settings.cameraJitterStrength));
			}
			ImGui::EndProperties();
		}
		if (enable && velocityTexture && ImGui::BeginTree("Show##ShowTAA"))
		{
			ImGui::Image(velocityTexture);
			ImGui::EndTree();
		}
		return result;
	}

	bool eeditor::editBloom(bool& enable, bloom_settings& settings, const ref<dx_texture>& bloomTexture)
	{
		bool result = false;
		if (ImGui::BeginProperties())
		{
			UNDOABLE_SETTING("enable bloom", enable,
				result |= ImGui::PropertyCheckbox("Enable bloom", enable));
			if (enable)
			{
				UNDOABLE_SETTING("bloom threshold", settings.threshold,
					result |= ImGui::PropertySlider("Bloom threshold", settings.threshold, 0.5f, 100.f));
				UNDOABLE_SETTING("bloom strength", settings.strength,
					result |= ImGui::PropertySlider("Bloom strength", settings.strength));
			}
			ImGui::EndProperties();
		}
		if (enable && bloomTexture && ImGui::BeginTree("Show##ShowBloom"))
		{
			ImGui::Image(bloomTexture);
			ImGui::EndTree();
		}
		return result;
	}

	bool eeditor::editSharpen(bool& enable, sharpen_settings& settings)
	{
		bool result = false;
		if (ImGui::BeginProperties())
		{
			UNDOABLE_SETTING("enable sharpen", enable,
				result |= ImGui::PropertyCheckbox("Enable sharpen", enable));
			if (enable)
			{
				UNDOABLE_SETTING("sharpen strength", settings.strength,
					result |= ImGui::PropertySlider("Sharpen strength", settings.strength));
			}
			ImGui::EndProperties();
		}
		return result;
	}

	volatile bool paused = false;

	void eeditor::forceStart()
	{
		this->scene->play();
		undoStacks[1].reset();
		setSelectedEntity({});

		//for (auto [entityHandle, rigidbody, transform] : scene->get_current_world()->group(components_group<physics::DynamicBodyComponent, TransformComponent>).each())
		//{
		//	rigidbody.manual_set_physics_position_and_rotation(transform.transform.position, transform.transform.rotation);
		//}

		//for (auto [entityHandle, rigidbody, transform] : scene->get_current_world()->group(components_group<physics::StaticBodyComponent, TransformComponent>).each())
		//{
		//	rigidbody.manual_set_physics_position_and_rotation(transform.transform.position, transform.transform.rotation);
		//}

		//if (!paused)
		//{
		//	app->linker->start();
		//}
		//else
		//{
		//	paused = false;
		//}
	}

	void eeditor::forcePause()
	{
		this->scene->pause();
		paused = true;
	}

	void eeditor::forceStop()
	{
		this->scene->stop();
		this->scene->environment.forceUpdate(this->scene->sun.direction);
		setSelectedEntity({});
		//app->linker->reload_src();
		//physics::PhysicsEngine::get_physics_core()->reset_actors_velocity_and_inertia();
		paused = false;
		this->scene->editor_camera.setPositionAndRotation(vec3(0.0f), quat::identity);
	}

	void eeditor::visualizePhysics(ldr_render_pass* ldrRenderPass) const
	{
		/*using namespace physx;

		if (!renderPhysicsShapes)
		{
			return;
		}

		auto scene = physics::PhysicsEngine::get_physics_core()->get_scene();
		const physx::PxRenderBuffer& rb = scene->getRenderBuffer();

		for (physx::PxU32 i = 0; i < rb.getNbPoints(); i++)
		{
			const physx::PxDebugPoint& point = rb.getPoints()[i];
			renderPoint(physx::createVec3(point.pos), vec4(1.0f), ldrRenderPass);
		}

		for (physx::PxU32 i = 0; i < rb.getNbLines(); i++)
		{
			const physx::PxDebugLine& line = rb.getLines()[i];
			renderLine(physx::createVec3(line.pos0), physx::createVec3(line.pos1), vec4(1.0f), ldrRenderPass);
		}*/
	}

	void eeditor::drawSettings(float dt)
	{
		ref<World> world = scene->get_current_world();

		if (ImGui::Begin("Settings"))
		{
			path_tracer& pathTracer = renderer->pathTracer;

			ImGui::Text("%.3f ms, %u FPS", dt * 1000.f, (uint32)(1.f / dt));

			if (ImGui::BeginProperties())
			{
				UNDOABLE_SETTING("time scale", this->scene->timestep_scale,
					ImGui::PropertySlider("Time scale", this->scene->timestep_scale));

				UNDOABLE_SETTING("renderer mode", renderer->mode,
					if (ImGui::PropertyDropdown("Renderer mode", rendererModeNames, renderer_mode_count, (uint32&)renderer->mode))
					{
						pathTracer.resetRendering();
					});

				dx_memory_usage memoryUsage = dxContext.getMemoryUsage();

				ImGui::PropertyValue("Video memory usage", "%u / %uMB", memoryUsage.currentlyUsed, memoryUsage.available);
				ImGui::PropertyValue("Render queue command lists count", "%u", dxContext.renderQueue.totalNumCommandLists);

				UNDOABLE_SETTING("aspect ratio", renderer->aspectRatioMode,
					ImGui::PropertyDropdown("Aspect ratio", aspectRatioNames, aspect_ratio_mode_count, (uint32&)renderer->aspectRatioMode));

				UNDOABLE_SETTING("static shadow map caching", enableStaticShadowMapCaching,
					ImGui::PropertyCheckbox("Static shadow map caching", enableStaticShadowMapCaching));

				UNDOABLE_SETTING("shadow map caching", renderer->settings.cacheShadowMap,
					ImGui::PropertyCheckbox("Shadow map caching", renderer->settings.cacheShadowMap));

				ImGui::EndProperties();
			}

			if (!this->scene->is_pausable())
			{
				editCamera(this->scene->camera);
			}
			else
			{
				editCamera(this->scene->editor_camera);
			}
			editTonemapping(renderer->settings.tonemapSettings);
			editSunShadowParameters(this->scene->sun);

			if (ImGui::BeginTree("Post processing"))
			{
				if (renderer->spec.allowAO) { editAO(renderer->settings.enableAO, renderer->settings.aoSettings, renderer->getAOResult()); ImGui::Separator(); }
				if (renderer->spec.allowSSS) { editSSS(renderer->settings.enableSSS, renderer->settings.sssSettings, renderer->getSSSResult()); ImGui::Separator(); }
				if (renderer->spec.allowSSR) { editSSR(renderer->settings.enableSSR, renderer->settings.ssrSettings, renderer->getSSRResult()); ImGui::Separator(); }
				if (renderer->spec.allowTAA) { editTAA(renderer->settings.enableTAA, renderer->settings.taaSettings, renderer->getScreenVelocities()); ImGui::Separator(); }
				if (renderer->spec.allowBloom) { editBloom(renderer->settings.enableBloom, renderer->settings.bloomSettings, renderer->getBloomResult()); ImGui::Separator(); }
				if (renderer->spec.allowDLSS)
				{

#if ENABLE_DLSS

					if (ImGui::BeginProperties())
					{
						// For now it's only fsr2 wrapper. I do not have rtx gpu now :(((
						ImGui::PropertyCheckbox("Enable FSR 2.0", renderer->settings.enableDLSS);

						ImGui::EndProperties();
					}

					if (renderer->settings.enableDLSS)
					{
						if (!renderer->dlssInited)
						{
							renderer->dlss_adapter.initialize(renderer);
						}
					}
					else
					{
						renderer->settings.tonemapSettings = renderer->settings.defaultTonemapSettings;
					}
					ImGui::Separator();

#endif

				}
				editSharpen(renderer->settings.enableSharpen, renderer->settings.sharpenSettings);

				ImGui::EndTree();
			}

			if (ImGui::BeginTree("Environment"))
			{
				auto& environment = this->scene->environment;

				if (ImGui::BeginProperties())
				{
					AssetHandle handle = (environment.sky) ? environment.sky->handle : AssetHandle{};
					if (ImGui::PropertyAssetHandle("Texture source", EDITOR_ICON_IMAGE_HDR, handle, "Make proc"))
					{
						if (handle)
						{
							environment.setFromTexture(get_path_from_asset_handle(handle));
						}
						else
						{
							environment.setToProcedural(this->scene->sun.direction);
						}
					}

					UNDOABLE_SETTING("sky intensity", environment.skyIntensity,
						ImGui::PropertySlider("Sky intensity", environment.skyIntensity, 0.f, 2.f));
					UNDOABLE_SETTING("GI intensity", environment.globalIlluminationIntensity,
						ImGui::PropertySlider("GI intensity", environment.globalIlluminationIntensity, 0.f, 2.f));

					environment_gi_mode tempMode = environment.giMode;

					ImGui::PropertyDropdown("GI mode", environmentGIModeNames, 1 + 1, (uint32&)tempMode);

					if (tempMode != environment.giMode)
					{
						struct change_gi_mode_data { environment_gi_mode mode; environment_gi_mode& destMode; } data{ tempMode, this->scene->environment.giMode };
						main_thread_job_queue.createJob<change_gi_mode_data>([](change_gi_mode_data& data, JobHandle job)
							{
								data.destMode = data.mode;
							}, data).submit_now();
					}

					ImGui::EndProperties();
				}

				if (environment.giMode == environment_gi_raytraced)
				{
					if (ImGui::BeginTree("Light probe"))
					{
						auto& grid = environment.lightProbeGrid;

						if (ImGui::BeginProperties())
						{
							UNDOABLE_SETTING("visualize probes", grid.visualizeProbes,
								ImGui::PropertyCheckbox("Visualize probes", grid.visualizeProbes));
							UNDOABLE_SETTING("visualize rays", grid.visualizeRays,
								ImGui::PropertyCheckbox("Visualize rays", grid.visualizeRays));

							UNDOABLE_SETTING("auto rotate rays", grid.autoRotateRays,
								ImGui::PropertyCheckbox("Auto rotate rays", grid.autoRotateRays));
							if (!grid.autoRotateRays)
							{
								grid.rotateRays = ImGui::PropertyButton("Rotate", "Go");
							}

							ImGui::EndProperties();
						}

						if (ImGui::BeginTree("Irradiance"))
						{
							if (ImGui::BeginProperties())
							{
								UNDOABLE_SETTING("irradiance UI scale", grid.irradianceUIScale,
									ImGui::PropertySlider("Scale", grid.irradianceUIScale, 0.1f, 20.f));
								ImGui::EndProperties();
							}
							if(grid.irradiance)
							{
								ImGui::Image(grid.irradiance, (uint32)(grid.irradiance->width * grid.irradianceUIScale));
							}
							ImGui::EndTree();
						}
						if (ImGui::BeginTree("Depth"))
						{
							if (ImGui::BeginProperties())
							{
								UNDOABLE_SETTING("depth UI scale", grid.depthUIScale,
									ImGui::PropertySlider("Scale", grid.depthUIScale, 0.1f, 20.f));
								ImGui::EndProperties();
							}
							if(grid.depth)
							{
								ImGui::Image(grid.depth, (uint32)(grid.depth->width * grid.depthUIScale));
							}
							ImGui::EndTree();
						}
						if (ImGui::BeginTree("Raytraced radiance"))
						{
							if (ImGui::BeginProperties())
							{
								UNDOABLE_SETTING("raytraced radiance UI scale", grid.raytracedRadianceUIScale,
									ImGui::PropertySlider("Scale", grid.raytracedRadianceUIScale, 0.1f, 20.f));
								ImGui::EndProperties();
							}
							if(grid.raytracedRadiance)
							{
								ImGui::Image(grid.raytracedRadiance, (uint32)(grid.raytracedRadiance->width * grid.raytracedRadianceUIScale));
							}
							ImGui::EndTree();
						}

						ImGui::EndTree();
					}
				}
				ImGui::EndTree();
			}

			/*if (ImGui::BeginTree("Physics (PhysX)"))
			{
				if (ImGui::BeginProperties())
				{
					const auto& physicsRef = physics::PhysicsEngine::get_physics_core();
					float frame_rate = physicsRef->frame_rate;

					ImGui::PropertyInput("Frame rate", frame_rate);
					if (frame_rate != physicsRef->frame_rate)
					{
						physicsRef->frame_rate = frame_rate;
					}

					if (physicsRef->frame_rate < 30)
					{
						ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.f, 0.f, 1.f));
						ImGui::PropertyValue("", "Low frame rate");
						ImGui::PopStyleColor();
					}

					uint32 nbaa = physicsRef->nb_active_actors.load(std::memory_order_relaxed);
					uint32 nba = physicsRef->actors_map.size();
					ImGui::PropertyValue("Number of active actors", std::to_string(nbaa).c_str());
					ImGui::PropertyValue("Number of actors", std::to_string(nba).c_str());
					if (this->scene->is_pausable())
					{
						ImGui::PropertyCheckbox("Render physics shapes", renderPhysicsShapes);
					}
#if PX_GPU_BROAD_PHASE
					ImGui::PropertyValue("Broad phase", "GPU");
#else
					ImGui::PropertyValue("Broad phase", "PABP (CPU)");
#endif
					ImGui::EndProperties();
				}

				ImGui::EndTree();
			}*/

			if (ImGui::BeginTree("Audio"))
			{
				bool change = false;
				if (ImGui::BeginProperties())
				{
					const float maxVolume = 3.f;
					UNDOABLE_SETTING("master volume", masterAudioSettings.volume,
						change |= ImGui::PropertySlider("Master volume", masterAudioSettings.volume, 0.f, maxVolume));

					ImGui::PropertySeparator();

					for (uint32 i = 0; i < sound_type_count; ++i)
					{
						UNDOABLE_SETTING(soundTypeNames[i], soundTypeVolumes[i],
							change |= ImGui::PropertySlider(soundTypeNames[i], soundTypeVolumes[i], 0.f, 1.f));
					}

					ImGui::PropertySeparator();

					UNDOABLE_SETTING("toggle reverb", masterAudioSettings.reverbEnabled,
						ImGui::PropertyCheckbox("Reverb enabled", masterAudioSettings.reverbEnabled));

					if (masterAudioSettings.reverbEnabled)
					{
						UNDOABLE_SETTING("reverb preset", masterAudioSettings.reverbPreset,
							change |= ImGui::PropertyDropdown("Reverb preset", reverbPresetNames, reverb_preset_count, (uint32&)masterAudioSettings.reverbPreset));
					}

					ImGui::EndProperties();
				}
				ImGui::EndTree();
			}

			if (renderer->mode == renderer_mode_pathtraced)
			{
				if (ImGui::BeginProperties())
				{
					auto& settings = pathTracer.settings;

					UNDOABLE_SETTING("max recursion depth", settings.recursionDepth,
						ImGui::PropertySlider("Max recursion depth", settings.recursionDepth, 0, settings.maxRecursionDepth - 1));
					UNDOABLE_SETTING("start russion roulette after", settings.startRussianRouletteAfter,
						ImGui::PropertySlider("Start russian roulette after", settings.startRussianRouletteAfter, 0, settings.recursionDepth));
					UNDOABLE_SETTING("use thin lens camera", settings.useThinLensCamera,
						ImGui::PropertyCheckbox("Use thin lens camera", settings.useThinLensCamera));
					if (settings.useThinLensCamera)
					{
						UNDOABLE_SETTING("focal length", settings.focalLength,
							ImGui::PropertySlider("Focal length", settings.focalLength, 0.5f, 50.f));
						UNDOABLE_SETTING("f-number", settings.fNumber,
							ImGui::PropertySlider("F-Number", settings.fNumber, 1.f, 128.f));
					}
					UNDOABLE_SETTING("use real materials", settings.useRealMaterials,
						ImGui::PropertyCheckbox("Use real materials", settings.useRealMaterials));
					UNDOABLE_SETTING("enable direct lighting", settings.enableDirectLighting,
						ImGui::PropertyCheckbox("Enable direct lighting", settings.enableDirectLighting));
					if (settings.enableDirectLighting)
					{
						UNDOABLE_SETTING("light intensity scale", settings.lightIntensityScale,
							ImGui::PropertySlider("Light intensity scale", settings.lightIntensityScale, 0.f, 50.f));
						UNDOABLE_SETTING("point light radius", settings.pointLightRadius,
							ImGui::PropertySlider("Point light radius", settings.pointLightRadius, 0.01f, 1.f));

						UNDOABLE_SETTING("multiple importance sampling", settings.multipleImportanceSampling,
							ImGui::PropertyCheckbox("Multiple importance sampling", settings.multipleImportanceSampling));
					}

					ImGui::EndProperties();
				}
			}
			else
			{
				if (ImGui::BeginTree("Particle systems"))
				{
					//editFireParticleSystem(app->fireParticleSystem);
					//editBoidParticleSystem(app->boidParticleSystem);

					ImGui::EndTree();
				}
			}
		}

		ImGui::End();
	}

	void editTexture(const char* name, ref<dx_texture>& tex, uint32 loadFlags)
	{
		AssetHandle asset = {};
		if (tex)
		{
			asset = tex->handle;
		}

		if (ImGui::PropertyTextureAssetHandle(name, EDITOR_ICON_IMAGE, asset, tex))
		{
			fs::path path = get_path_from_asset_handle(asset);
			fs::path relative = fs::relative(path, fs::current_path());
			if (auto newTex = loadTextureFromFileAsync(path, loadFlags))
			{
				tex = newTex;
			}
		}
	}

	void editMesh(const char* name, ref<MultiMesh>& mesh, uint32 loadFlags)
	{
		AssetHandle asset = {};
		if (mesh)
		{
			asset = mesh->handle;
		}

		if (ImGui::PropertyAssetHandle(name, EDITOR_ICON_MESH, asset))
		{
			fs::path path = get_path_from_asset_handle(asset);
			fs::path relative = fs::relative(path, fs::current_path());
			if (auto newMesh = import_mesh_from_file(path, loadFlags))
			{
				mesh = newMesh;
			}
		}
	}

	void editMaterial(const ref<pbr_material>& material)
	{
		if (ImGui::BeginProperties())
		{
			AssetHandle dummy = {};

			editTexture("Albedo", material->albedo, image_load_flags_default);
			editTexture("Normal", material->normal, image_load_flags_default_noncolor);
			editTexture("Roughness", material->roughness, image_load_flags_default_noncolor);
			editTexture("Metallic", material->metallic, image_load_flags_default_noncolor);

			ImGui::PropertyColor("Emission", material->emission);
			ImGui::PropertyColor("Albedo tint", material->albedoTint);
			ImGui::PropertyDropdown("Shader", pbrMaterialShaderNames, pbr_material_shader_count, (uint32&)material->shader);
			ImGui::PropertySlider("UV scale", material->uvScale, 0.0f, 150.0f);
			ImGui::PropertySlider("Translucency", material->translucency);

			if (!material->roughness)
			{
				ImGui::PropertySlider("Roughness override", material->roughnessOverride);
			}

			if (!material->metallic)
			{
				ImGui::PropertySlider("Metallic override", material->metallicOverride);
			}

			ImGui::EndProperties();
		}
	}

	void editSubmeshTransform(trs* transform)
	{
		ImGui::Drag("Position", transform->position, 0.1f);
		vec3 selectedEntityEulerRotation = quat_to_euler(transform->rotation);

		if (ImGui::Drag("Rotation", selectedEntityEulerRotation, 0.1f))
		{
			transform->rotation = euler_to_quat(selectedEntityEulerRotation);
		}

		ImGui::Drag("Scale", transform->scale, 0.1f);
	}

}