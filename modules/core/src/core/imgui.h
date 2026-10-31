// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core_api.h"

#include "core/math.h"

#include "dx/dx_command_list.h"
#include "dx/dx_texture.h"

#pragma warning(push, 0)

#define IM_ASSERT(condition) ASSERT(condition)
#include <imgui/imgui.h>
#include <fontawesome/IconsFontAwesome5.h>

#pragma warning(pop)

#define IMGUI_ICON_COLS 4
#define IMGUI_ICON_ROWS 4

#define IMGUI_ICON_DEFAULT_SIZE 35
#define IMGUI_ICON_DEFAULT_SPACING 3.f

namespace era_engine
{
    ImGuiContext* initialize_imgui(struct DXWindow& window);
    void new_imgui_frame(float dt);

    void end_imgui_frame();

    // Draws the main viewport of the current frame.
    void render_imgui(struct DXCommandList* cl);

    LRESULT handle_imgui_input(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param);

    struct DXTexture;
    struct AssetHandle;

    enum ImGuiIcon
    {
        IMGUI_ICON_GLOBAL,
        IMGUI_ICON_LOCAL,
        IMGUI_ICON_TRANSLATE,
        IMGUI_ICON_ROTATE,
        IMGUI_ICON_SCALE,
        IMGUI_ICON_CROSS,
        IMGUI_ICON_PLAY,
        IMGUI_ICON_STOP,
        IMGUI_ICON_PAUSE,
    };

    static const char* imgui_icon_names[] =
        {
            "Transform in global coordinate system (G)",
            "Transform in local coordinate system (G)",
            "Translate (W)",
            "Rotate (E)",
            "Scale (R)",
            "No gizmo (Q)",
            "Play",
            "Stop",
            "Pause",
    };

}

namespace ImGui
{
    using namespace era_engine;

    ERA_CORE_API bool AnyModifiersDown();

    ERA_CORE_API bool IsItemActiveLastFrame();

    ERA_CORE_API bool BeginWindowHiddenTabBar(const char* name, bool* open = 0, ImGuiWindowFlags flags = 0);
    ERA_CORE_API bool BeginControlsWindow(const char* name, ImVec2 parent_relative_offset, ImVec2 parent_absolute_offset);

    void Image(const ref<DXTexture>& texture, uint32 width = 0, uint32 height = 0, ImVec2 uv0 = ImVec2(0.f, 0.f), ImVec2 uv1 = ImVec2(1.f, 1.f));

    NODISCARD inline float CalcButtonWidth(const char* text)
    {
        return CalcTextSize(text).x + (GetStyle().FramePadding.x + GetStyle().FrameBorderSize) * 2.f;
    }

    bool ImageButton(const ref<DXTexture>& texture, uint32 width, uint32 height, ImVec2 uv_top_left = ImVec2(0, 0), ImVec2 uv_bottom_right = ImVec2(1, 1));

    ERA_CORE_API void Icon(ImGuiIcon icon, uint32 size);
    ERA_CORE_API bool IconButton(uint32 id, ImGuiIcon icon, uint32 size, bool enabled = true);
    ERA_CORE_API bool IconRadioButton(ImGuiIcon icon, int* current, int value, uint32 size, bool enabled = true);

    ERA_CORE_API bool Dropdown(const char* label, const char** names, uint32 count, uint32& current);
    ERA_CORE_API bool Dropdown(const char* label, const char* (*name_func)(uint32, void*), uint32& current, void* data = 0);

    ERA_CORE_API bool DisableableButton(const char* label, bool enabled);
    ERA_CORE_API bool DisableableCheckbox(const char* label, bool& v, bool enabled);

    ERA_CORE_API bool SelectableWrapped(const char* label, int width, bool selected = false, ImGuiSelectableFlags flags = 0);

    ERA_CORE_API bool BeginTree(const char* label, bool default_open = false);
    ERA_CORE_API bool BeginTreeColoredText(const char* label, vec3 color, bool default_open = false);
    ERA_CORE_API void EndTree();

    inline void Value(const char* prefix, int64 v)
    {
        ImGui::Text("%s: %lld", prefix, v);
    }
    inline void Value(const char* prefix, uint64 v)
    {
        ImGui::Text("%s: %llu", prefix, v);
    }
    inline void Value(const char* prefix, const char* v)
    {
        ImGui::Text("%s: %s", prefix, v);
    }

    ERA_CORE_API void CenteredText(const char* text);

    ERA_CORE_API void PopupOkButton(uint32 width = 120);

    ERA_CORE_API bool AssetHandle(const char* label, const char* type, era_engine::AssetHandle& asset, const char* clear_text = 0);
    bool TextureAssetHandle(const char* label, const char* type, era_engine::AssetHandle& asset,
        const ref<DXTexture>& texture, uint32 width = 0, uint32 height = 0, ImVec2 uv0 = ImVec2(0.f, 0.f), ImVec2 uv1 = ImVec2(1.f, 1.f), const char* clear_text = 0);

    ERA_CORE_API bool Drag(const char* label, float& f, float speed = 1.f, float min = -FLT_MAX, float max = FLT_MAX, const char* format = "%.3f");
    ERA_CORE_API bool Drag(const char* label, vec2& f, float speed = 1.f, float min = -FLT_MAX, float max = FLT_MAX, const char* format = "%.3f");
    ERA_CORE_API bool Drag(const char* label, vec3& f, float speed = 1.f, float min = -FLT_MAX, float max = FLT_MAX, const char* format = "%.3f");
    ERA_CORE_API bool Drag(const char* label, vec4& f, float speed = 1.f, float min = -FLT_MAX, float max = FLT_MAX, const char* format = "%.3f");
    ERA_CORE_API bool Drag(const char* label, int32& f, float speed = 1.f, int32 min = INT32_MIN, int32 max = INT32_MAX, const char* format = "%d");
    ERA_CORE_API bool Drag(const char* label, uint32& f, float speed = 1.f, uint32 min = 0, uint32 max = UINT32_MAX, const char* format = "%u");
    ERA_CORE_API bool DragRange(const char* label, float& lo, float& hi, float speed = 1.f, float min = -FLT_MAX, float max = FLT_MAX, const char* format = "%.3f");
    ERA_CORE_API bool DragRange(const char* label, int32& lo, int32& hi, float speed = 1.f, int32 min = INT32_MIN, int32 max = INT32_MAX, const char* format = "%.3f");

    ERA_CORE_API bool BeginProperties();
    ERA_CORE_API void EndProperties();

    ERA_CORE_API void PropertyValue(const char* label, const char* format, ...);

    ERA_CORE_API inline void PropertyValue(const char* label, bool v)
    {
        ImGui::PropertyValue(label, v ? "True" : "False");
    }
    ERA_CORE_API inline void PropertyValue(const char* label, float v, const char* format = "%.3f")
    {
        ImGui::PropertyValue(label, format, v);
    }
    ERA_CORE_API inline void PropertyValue(const char* label, int32 v, const char* format = "%d")
    {
        ImGui::PropertyValue(label, format, v);
    }
    ERA_CORE_API inline void PropertyValue(const char* label, uint32 v, const char* format = "%u")
    {
        ImGui::PropertyValue(label, format, v);
    }
    ERA_CORE_API inline void PropertyValue(const char* label, int64 v, const char* format = "%lld")
    {
        ImGui::PropertyValue(label, format, v);
    }
    ERA_CORE_API inline void PropertyValue(const char* label, uint64 v, const char* format = "%llu")
    {
        ImGui::PropertyValue(label, format, v);
    }
    ERA_CORE_API inline void PropertyValue(const char* label, vec2 v, const char* format = "%.3f, %.3f")
    {
        ImGui::PropertyValue(label, format, v.x, v.y);
    }
    ERA_CORE_API inline void PropertyValue(const char* label, vec3 v, const char* format = "%.3f, %.3f, %.3f")
    {
        ImGui::PropertyValue(label, format, v.x, v.y, v.z);
    }
    ERA_CORE_API inline void PropertyValue(const char* label, vec4 v, const char* format = "%.3f, %.3f, %.3f, %.3f")
    {
        ImGui::PropertyValue(label, format, v.x, v.y, v.z, v.w);
    }

    ERA_CORE_API bool PropertyCheckbox(const char* label, bool& v);

    ERA_CORE_API bool PropertySlider(const char* label, float& f, float min_value = 0.f, float max_value = 1.f, const char* format = "%.3f", ImGuiSliderFlags flags = ImGuiSliderFlags_None);
    ERA_CORE_API bool PropertySlider(const char* label, vec2& f, float min_value = 0.f, float max_value = 1.f, const char* format = "%.3f", ImGuiSliderFlags flags = ImGuiSliderFlags_None);
    ERA_CORE_API bool PropertySlider(const char* label, vec3& f, float min_value = 0.f, float max_value = 1.f, const char* format = "%.3f", ImGuiSliderFlags flags = ImGuiSliderFlags_None);
    ERA_CORE_API bool PropertySlider(const char* label, vec4& f, float min_value = 0.f, float max_value = 1.f, const char* format = "%.3f", ImGuiSliderFlags flags = ImGuiSliderFlags_None);

    ERA_CORE_API bool PropertySliderAngle(const char* label, float& f_rad, float min_value_deg = -360.f, float max_value_deg = 360.f, const char* format = "%.0f deg");

    ERA_CORE_API bool PropertySlider(const char* label, int32& v, int min_value, int max_value, const char* format = "%d");
    ERA_CORE_API bool PropertySlider(const char* label, uint32& v, uint32 min_value, uint32 max_value, const char* format = "%u");

    ERA_CORE_API bool PropertyInput(const char* label, float& f, const char* format = "%.3f");
    ERA_CORE_API bool PropertyInput(const char* label, vec2& f, const char* format = "%.3f");
    ERA_CORE_API bool PropertyInput(const char* label, vec3& f, const char* format = "%.3f");
    ERA_CORE_API bool PropertyInput(const char* label, vec4& f, const char* format = "%.3f");
    ERA_CORE_API bool PropertyInput(const char* label, int32& f, const char* format = "%d");
    ERA_CORE_API bool PropertyInput(const char* label, uint32& f, const char* format = "%u");

    ERA_CORE_API bool PropertyDrag(const char* label, float& f, float speed = 1.f, float min = -FLT_MAX, float max = FLT_MAX, const char* format = "%.3f");
    ERA_CORE_API bool PropertyDrag(const char* label, vec2& f, float speed = 1.f, float min = -FLT_MAX, float max = FLT_MAX, const char* format = "%.3f");
    ERA_CORE_API bool PropertyDrag(const char* label, vec3& f, float speed = 1.f, float min = -FLT_MAX, float max = FLT_MAX, const char* format = "%.3f");
    ERA_CORE_API bool PropertyDrag(const char* label, vec4& f, float speed = 1.f, float min = -FLT_MAX, float max = FLT_MAX, const char* format = "%.3f");
    ERA_CORE_API bool PropertyDrag(const char* label, int32& f, float speed = 1.f, int32 min = INT32_MIN, int32 max = INT32_MAX, const char* format = "%d");
    ERA_CORE_API bool PropertyDrag(const char* label, uint32& f, float speed = 1.f, uint32 min = 0, uint32 max = UINT32_MAX, const char* format = "%u");
    ERA_CORE_API bool PropertyDragRange(const char* label, float& lo, float& hi, float speed = 1.f, float min = -FLT_MAX, float max = FLT_MAX, const char* format = "%.3f");
    ERA_CORE_API bool PropertyDragRange(const char* label, int32& lo, int32& hi, float speed = 1.f, int32 min = INT32_MIN, int32 max = INT32_MAX, const char* format = "%.3f");

    ERA_CORE_API bool PropertyDropdown(const char* label, const char** names, uint32 count, uint32& current);
    ERA_CORE_API bool PropertyDropdown(const char* label, const char* (*name_func)(uint32, void*), uint32& current, void* data = 0);
    ERA_CORE_API bool PropertyDropdownPowerOfTwo(const char* label, uint32 from, uint32 to, uint32& current);
    ERA_CORE_API bool PropertyColor(const char* label, vec3& f);
    ERA_CORE_API bool PropertyColor(const char* label, vec4& f);

    ERA_CORE_API bool PropertyColorWheel(const char* label, vec3& f);
    ERA_CORE_API bool PropertyColorWheel(const char* label, vec4& f);

    ERA_CORE_API bool PropertyButton(const char* label, const char* button_text, const char* hover_text = 0, ImVec2 size = ImVec2(0, 0));

    ERA_CORE_API bool PropertyInputText(const char* label, char* buffer, uint32 buffer_size, bool disable_input = false);

    ERA_CORE_API bool PropertyAssetHandle(const char* label, const char* type, era_engine::AssetHandle& asset, const char* clear_text = 0);
    bool PropertyTextureAssetHandle(const char* label, const char* type, era_engine::AssetHandle& asset,
        const ref<DXTexture>& texture, uint32 width = 0, uint32 height = 0, ImVec2 uv0 = ImVec2(0.f, 0.f), ImVec2 uv1 = ImVec2(1.f, 1.f), const char* clear_text = 0);

    ERA_CORE_API bool PropertyDragDropStringTarget(const char* label, const char* drag_drop_id, std::string& value, const char* clear_label = 0);

    ERA_CORE_API void PropertySeparator();

    ERA_CORE_API bool Spline(const char* label, ImVec2 size, uint32 max_num_points, float* x, float* y, uint32 draw_resolution = 256);

    template <uint32 max_num_points>
    bool Spline(const char* label, ImVec2 size, catmull_rom_spline<float, max_num_points>& s, uint32 draw_resolution = 256)
    {
        return Spline(label, size, max_num_points, s.ts, s.values, draw_resolution);
    }

    bool PropertySpline(const char* label, uint32 max_num_points, float* x, float* y, uint32 draw_resolution = 256);

    template <uint32 max_num_points>
    bool PropertySpline(const char* label, catmull_rom_spline<float, max_num_points>& s, uint32 draw_resolution = 256)
    {
        return PropertySpline(label, max_num_points, s.ts, s.values, draw_resolution);
    }

    static const ImColor white(1.f, 1.f, 1.f, 1.f);
    static const ImColor yellow(1.f, 1.f, 0.f, 1.f);
    static const ImColor green(0.f, 1.f, 0.f, 1.f);
    static const ImColor red(1.f, 0.f, 0.f, 1.f);
    static const ImColor blue(0.f, 0.f, 1.f, 1.f);


#define IM_ARRAYSIZE(_ARR) ((int)(sizeof(_ARR) / sizeof(*_ARR)))

    template <int valuesCount = 90>
    class PlotLinesInstance
    {
    public:
        PlotLinesInstance()
        {
            memset(values, 0, sizeof(float) * valuesCount);
        }

        void plot(const char* label, float new_value, const char* overlay_text, float scale_min = FLT_MAX, float scale_max = FLT_MAX, ImVec2 graph_size = ImVec2(0, 80))
        {
            for (; ImGui::GetTime() > time + 1.0f / 60.0f; time += 1.0f / 60.0f)
            {
                values[offset] = new_value;
                offset = (offset + 1) % valuesCount;
            }
            ImGui::PlotLines(label, values, valuesCount, offset, overlay_text, scale_min, scale_max, graph_size);
        }

    private:
        float values[valuesCount];
        float time = ImGui::GetTime();
        int offset;
    };
}