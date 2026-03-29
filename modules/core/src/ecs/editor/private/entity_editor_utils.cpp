#include "ecs/editor/entity_editor_utils.h"
#include "ecs/world.h"
#include "ecs/component.h"
#include "ecs/base_components/base_components.h"

#include "imgui/imgui_internal.h"

namespace era_engine
{
	void EntityEditorUtils::edit_entity(World* world, const Entity::Handle handle)
	{
		ImGui::AlignTextToFramePadding();

		ImGui::PushID((uint32)handle);

		entt::registry& registry = world->get_registry();
		for (auto&& curr : registry.storage())
		{
			entt::id_type cid = curr.first;
			auto& storage = curr.second;
			entt::type_info ctype = storage.type();

			if (storage.contains(handle))
			{
				const Component* comp = static_cast<const Component*>(registry.storage(cid)->get(handle));

				rttr::type type = comp->get_type();
				const char* comp_name = type.get_name().data();
				const ImGuiTreeNodeFlags treeNodeFlags = ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_AllowItemOverlap | ImGuiTreeNodeFlags_FramePadding;

				ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2{ 4, 4 });
				bool open = ImGui::TreeNodeEx(comp_name, treeNodeFlags, comp_name);
				ImGui::PopStyleVar();
				if (open)
				{
					for (auto& prop : type.get_properties())
					{
						const rttr::type prop_type = prop.get_type();
						if (prop_type.is_arithmetic())
						{
							if (prop_type == rttr::type::get<bool>())
							{
								auto value = prop.get_value(*comp).to_bool();
								if (ImGui::Checkbox(prop.get_name().data(), &value))
								{
									prop.set_value(*comp, value);
								}
							}
							else if (prop_type == rttr::type::get<char>())
							{
								auto value = prop.get_value(*comp).to_bool();
								if (ImGui::Checkbox(prop.get_name().data(), &value))
								{
									prop.set_value(*comp, (char)value);
								}
							}
							else if (prop_type == rttr::type::get<int8_t>())
							{
								auto value = prop.get_value(*comp).to_int();
								if (ImGui::InputInt(prop.get_name().data(), &value))
								{
									prop.set_value(*comp, (int8_t)value);
								}
							}
							else if (prop_type == rttr::type::get<int16_t>())
							{
								auto value = prop.get_value(*comp).to_int();
								if (ImGui::InputInt(prop.get_name().data(), &value))
								{
									prop.set_value(*comp, (int16_t)value);
								}
							}
							else if (prop_type == rttr::type::get<int32_t>())
							{
								auto value = prop.get_value(*comp).to_int();
								if (ImGui::InputInt(prop.get_name().data(), &value))
								{
									prop.set_value(*comp, (int32_t)value);
								}
							}
							else if (prop_type == rttr::type::get<int64_t>())
							{
								auto value = prop.get_value(*comp).to_int();
								if (ImGui::InputInt(prop.get_name().data(), &value))
								{
									prop.set_value(*comp, (int64_t)value);
								}
							}
							else if (prop_type == rttr::type::get<uint8_t>())
							{
								auto value = prop.get_value(*comp).to_int();
								if (ImGui::InputInt(prop.get_name().data(), &value))
								{
									prop.set_value(*comp, (uint8_t)value);
								}
							}
							else if (prop_type == rttr::type::get<uint16_t>())
							{
								auto value = prop.get_value(*comp).to_int();
								if (ImGui::InputInt(prop.get_name().data(), &value))
								{
									prop.set_value(*comp, (uint16_t)value);
								}
							}
							else if (prop_type == rttr::type::get<uint32_t>())
							{
								auto value = prop.get_value(*comp).to_int();
								if (ImGui::InputInt(prop.get_name().data(), &value))
								{
									prop.set_value(*comp, (uint32_t)value);
								}
							}
							else if (prop_type == rttr::type::get<uint64_t>())
							{
								auto value = prop.get_value(*comp).to_int();
								if (ImGui::InputInt(prop.get_name().data(), &value))
								{
									prop.set_value(*comp, (uint64_t)value);
								}
							}
							else if (prop_type == rttr::type::get<float>())
							{
								auto value = prop.get_value(*comp).to_float();
								if (ImGui::InputFloat(prop.get_name().data(), &value))
								{
									prop.set_value(*comp, value);
								}
							}
							else if (prop_type == rttr::type::get<double>())
							{
								auto value = prop.get_value(*comp).to_double();
								if (ImGui::InputDouble(prop.get_name().data(), &value))
								{
									prop.set_value(*comp, value);
								}
							}
						}
						else if (prop_type == rttr::type::get<trs>())
						{
							trs value = prop.get_value(*comp).get_value<trs>();
							show_vector3("Position", value.position, 1080);
							ImGui::SameLine();
							vec3 rot = quat_to_euler(value.rotation);
							show_vector3("Rotation", rot, 1080);
							value.rotation = euler_to_quat(rot);
							ImGui::SameLine();
							show_vector3("Scale", value.scale, 1080);
							prop.set_value(*comp, value);
						}
						else if (prop_type.is_enumeration())
						{
							ImGui::Text("Enum: %s. Value: %s", prop.get_name().data(), prop.get_enumeration().value_to_name({prop.get_value(*comp)}).data());
						}
						else if (prop_type == rttr::type::get<std::string>() || prop_type == rttr::type::get<char*>())
						{
							auto value = prop.get_value(*comp).to_string();
							char buf[64];
							strcpy_s(buf, value.data());
							if (ImGui::InputText(prop.get_name().data(), value.data(), 256))
							{
								prop.set_value(*comp, buf);
							}
						}
						else
						{
							ImGui::Text("Property: %s.", prop.get_name().data());
						}
					}
					ImGui::TreePop();
				}
			}
		}

		ImGui::PopID();
	}

	void EntityEditorUtils::show_vector3(const char* label, vec3& vector, float wdt)
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
	}

	void EntityEditorUtils::draw_float_wrap(float wdt, const char* label, float* v, float v_speed, float v_min, float v_max, const char* format, const ImGuiSliderFlags flags)
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
}