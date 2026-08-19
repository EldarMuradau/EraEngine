// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "core/log.h"
#include "core/imgui.h"
#include "core/memory.h"

#include "ecs/update_groups.h"

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
	bool log_window_open = true;
}

#ifdef ENABLE_MESSAGE_LOG

#define MAX_NB_MESSAGES 512

namespace era_engine
{
	struct LogMessage
	{
		const char* text;
		MessageType type;
		float lifetime;
		const char* file;
		const char* function;
		uint32 line;
	};

	static const ImVec4 color_per_type[] =
	{
		ImVec4(1.f, 1.f, 1.f, 1.f),
		ImVec4(1.f, 1.f, 0.f, 1.f),
		ImVec4(1.f, 0.f, 0.f, 1.f),
	};

	static Allocator arena;
	static std::vector<LogMessage> messages;
	static std::mutex mutex;

	static inline bool set_scroll_to_bottom = false;

	void log_message_internal(MessageType type, const char* file, const char* function, uint32 line, const char* format, ...)
	{
		std::lock_guard lock{ mutex };
		arena.ensure_free_size(1024);

		char* buffer = (char*)arena.get_current();

		va_list args;
		va_start(args, format);
		int count_written = vsnprintf(buffer, 1024, format, args);
		va_end(args);

		messages.push_back({ buffer, type, 5.f, file, function, line });
		std::cout << messages.back().text << std::endl;

		set_scroll_to_bottom = true;

		arena.set_current_to(buffer + count_written + 1);
	}

	void log_message(MessageType type, const char* format, ...)
	{
		std::lock_guard lock{ mutex };
		arena.ensure_free_size(KB(1));

		char* buffer = (char*)arena.get_current();

		va_list args;
		va_start(args, format);
		int count_written = vsnprintf(buffer, KB(1), format, args);
		va_end(args);

		messages.push_back({ buffer, type, 5.f, nullptr, nullptr, 0 });
		std::cout << messages.back().text << std::endl;

		set_scroll_to_bottom = true;

		arena.set_current_to(buffer + count_written + 1);
	}

	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<LogSystem>("LogSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr)
			.method("update", &System::update)(metadata("update_group", update_types::GAMEPLAY_NORMAL));
	}

	LogSystem::LogSystem(World* _world)
		: System(_world)
	{
		arena.initialize(0, MB(128));
	}

	LogSystem::~LogSystem()
	{
	}

	void LogSystem::init()
	{
	}

	void LogSystem::update(float dt)
	{
		dt = min(dt, 1.0f); // If the app hangs, we don't want all the messages to go missing.

		ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 10.f);
		ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.f, 0.1f));
		ImGui::SetNextWindowSize(ImVec2(0.f, 0.f)); // Auto-resize to content.
		bool window_open = ImGui::Begin("##Console", 0,
			ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
			ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoBringToFrontOnFocus);
		ImGui::PopStyleVar(2);

		uint32 count = (uint32)messages.size();
		uint32 start_index = 0;

		for (uint32 i = count - 1; i != UINT32_MAX; --i)
		{
			auto& msg = messages[i];

			if (msg.lifetime <= 0.f)
			{
				start_index = i + 1;
				break;
			}
			msg.lifetime -= dt;
		}

		uint32 num_messages_to_show = count - start_index;
		num_messages_to_show = min(num_messages_to_show, 8u);
		start_index = count - num_messages_to_show;

		ImGui::End();

		if (log_window_open)
		{
			if (ImGui::Begin(ICON_FA_CLIPBOARD_LIST "  Console", &log_window_open))
			{
				for (uint32 i = 0; i < count; ++i)
				{
					const auto& msg = messages[i];
					if (msg.file)
					{
						ImGui::TextColored(color_per_type[msg.type], "%s (%s [%u])", msg.text, msg.function, msg.line);
					}
					else
					{
						ImGui::TextColored(color_per_type[msg.type], "%s", msg.text);
					}
				}

				if (set_scroll_to_bottom)
				{
					ImGui::SetScrollHereY(1.0f);
					set_scroll_to_bottom = false;
				}
			}
			ImGui::End();
		}

		if (messages.size() > MAX_NB_MESSAGES)
		{
			messages.clear();
			arena.reset();
		}
	}
}

#endif
