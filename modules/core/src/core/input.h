#pragma once

#include "core_api.h"

namespace era_engine
{
    enum class MouseMode
    {
        Visible,        // Normal cursor, free movement
        Hidden,         // Cursor hidden but not captured
        Captured,       // Cursor hidden and locked to center
        CapturedRaw     // Captured with raw input
    };

    enum key_code : uint32
    {
        key_none = 0x00,
        key_left_mouse = VK_LBUTTON,
        key_right_mouse = VK_RBUTTON,
        key_middle_mouse = VK_MBUTTON,
        key_x1_mouse = VK_XBUTTON1,
        key_x2_mouse = VK_XBUTTON2,
        key_backspace = VK_BACK,
        key_tab = VK_TAB,
        key_enter = VK_RETURN,
        key_shift = VK_SHIFT,
        key_ctrl = VK_CONTROL,
        key_alt = VK_MENU,
        key_pause = VK_PAUSE,
        key_caps = VK_CAPITAL,
        key_esc = VK_ESCAPE,
        key_space = VK_SPACE,
        key_page_up = VK_PRIOR,
        key_page_down = VK_NEXT,
        key_end = VK_END,
        key_home = VK_HOME,
        key_left = VK_LEFT,
        key_up = VK_UP,
        key_right = VK_RIGHT,
        key_down = VK_DOWN,
        key_print = VK_SNAPSHOT,
        key_insert = VK_INSERT,
        key_delete = VK_DELETE,
        key_0 = '0', key_1 = '1', key_2 = '2', key_3 = '3', key_4 = '4',
        key_5 = '5', key_6 = '6', key_7 = '7', key_8 = '8', key_9 = '9',
        key_a = 'A', key_b = 'B', key_c = 'C', key_d = 'D', key_e = 'E',
        key_f = 'F', key_g = 'G', key_h = 'H', key_i = 'I', key_j = 'J',
        key_k = 'K', key_l = 'L', key_m = 'M', key_n = 'N', key_o = 'O',
        key_p = 'P', key_q = 'Q', key_r = 'R', key_s = 'S', key_t = 'T',
        key_u = 'U', key_v = 'V', key_w = 'W', key_x = 'X', key_y = 'Y',
        key_z = 'Z',
        key_numpad_0 = VK_NUMPAD0, key_numpad_1 = VK_NUMPAD1,
        key_numpad_2 = VK_NUMPAD2, key_numpad_3 = VK_NUMPAD3,
        key_numpad_4 = VK_NUMPAD4, key_numpad_5 = VK_NUMPAD5,
        key_numpad_6 = VK_NUMPAD6, key_numpad_7 = VK_NUMPAD7,
        key_numpad_8 = VK_NUMPAD8, key_numpad_9 = VK_NUMPAD9,
        key_multiply = VK_MULTIPLY, key_add = VK_ADD,
        key_subtract = VK_SUBTRACT, key_decimal = VK_DECIMAL,
        key_divide = VK_DIVIDE,
        key_f1 = VK_F1, key_f2 = VK_F2, key_f3 = VK_F3, key_f4 = VK_F4,
        key_f5 = VK_F5, key_f6 = VK_F6, key_f7 = VK_F7, key_f8 = VK_F8,
        key_f9 = VK_F9, key_f10 = VK_F10, key_f11 = VK_F11, key_f12 = VK_F12,
        key_num_lock = VK_NUMLOCK, key_scroll_lock = VK_SCROLL,
        key_left_shift = VK_LSHIFT, key_right_shift = VK_RSHIFT,
        key_left_ctrl = VK_LCONTROL, key_right_ctrl = VK_RCONTROL,
        key_left_alt = VK_LMENU, key_right_alt = VK_RMENU,
        key_semicolon = VK_OEM_1, key_plus = VK_OEM_PLUS,
        key_comma = VK_OEM_COMMA, key_minus = VK_OEM_MINUS,
        key_period = VK_OEM_PERIOD, key_slash = VK_OEM_2,
        key_tilde = VK_OEM_3, key_left_bracket = VK_OEM_4,
        key_backslash = VK_OEM_5, key_right_bracket = VK_OEM_6,
        key_quote = VK_OEM_7,
        key_count = 256
    };

    struct ERA_CORE_API InputKey
    {
        bool down = false;
        bool press_event = false;
        bool release_event = false;
        float held_time = 0.0f;
    };

    struct ERA_CORE_API InputMouseButton
    {
        bool down = false;
        bool click_event = false;
        bool double_click_event = false;
        bool release_event = false;
        float held_time = 0.0f;
        uint32 click_count = 0;
    };

    struct ERA_CORE_API MouseInput
    {
        InputMouseButton left;
        InputMouseButton right;
        InputMouseButton middle;
        InputMouseButton x1;
        InputMouseButton x2;
        float scroll = 0.0f;
        float scroll_delta = 0.0f;

        int32 x = 0;
        int32 y = 0;
        int32 dx = 0;
        int32 dy = 0;

        // Raw input deltas
        float raw_dx = 0.0f;
        float raw_dy = 0.0f;

        float relX = 0.0f;
        float relY = 0.0f;
        float reldx = 0.0f;
        float reldy = 0.0f;

        bool visible = true;
        bool captured = false;
    };

    struct ERA_CORE_API UserInput
    {
        InputKey keyboard[key_count];
        MouseInput mouse;
        bool over_window = false;
        bool window_focused = false;
        bool any_key_pressed = false;
        float padding[2];

        UserInput() = default;
        UserInput(const UserInput& other)
        {
            memcpy(keyboard, other.keyboard, sizeof(keyboard));
            mouse = other.mouse;
            over_window = other.over_window;
            window_focused = other.window_focused;
            any_key_pressed = other.any_key_pressed;
            padding[0] = other.padding[0];
            padding[1] = other.padding[1];
        }

        UserInput& operator=(const UserInput& other)
        {
            if (this != &other)
            {
                memcpy(keyboard, other.keyboard, sizeof(keyboard));
                mouse = other.mouse;
                over_window = other.over_window;
                window_focused = other.window_focused;
                any_key_pressed = other.any_key_pressed;
                padding[0] = other.padding[0];
                padding[1] = other.padding[1];
            }
            return *this;
        }
    };

    struct ERA_CORE_API InputConfig
    {
        // Mouse settings
        float mouse_sensitivity = 0.15f;
        float scroll_sensitivity = 0.5f;
        bool invert_y = false;
        bool use_raw_input = true;

        // Mouse behavior
        bool auto_capture_on_focus = true;
        bool right_click_to_capture = true;
        bool show_cursor_after_idle = true;
        float idle_time_before_show = 3.0f;

        // Key bindings
        uint32 key_toggle_mouse = key_esc;
        uint32 key_toggle_cursor = key_tab;
        uint32 key_capture = key_right_mouse;

        // Camera settings
        float camera_distance = 5.0f;
        float camera_min_distance = 2.0f;
        float camera_max_distance = 15.0f;
        float camera_smooth_speed = 5.0f;
        float camera_pitch_limit = 89.0f;
    };

    ERA_CORE_API const UserInput& get_current_frame_input();
}