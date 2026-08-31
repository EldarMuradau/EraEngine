#include "core/ecs/private/input_system.h"
#include "core/ecs/input_sender_component.h"
#include "core/ecs/input_receiver_component.h"
#include "core/imgui.h"
#include "core/log.h"
#include "core/cpu_profiling.h"

#include "engine/engine.h"

#include "window/dx_window.h"

#include "rendering/main_renderer.h"
#include "rendering/ecs/renderer_holder_root_component.h"

#include "ecs/update_groups.h"
#include "ecs/base_components/transform_component.h"

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui/imgui_internal.h>

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
	const UserInput& get_current_frame_input()
	{
		dx_window* window = get_object<dx_window>();
		return window->get_current_frame_input();
	}

	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<InputSystem>("InputSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("base")))
			.method("update", &InputSystem::update)(metadata("update_group", update_types::INPUT));
	}

	InputSystem::InputSystem(World* _world)
		: System(_world)
	{
		renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();
		ASSERT(renderer_holder_rc != nullptr);
	}

	InputSystem::~InputSystem()
	{
	}

	void InputSystem::init()
	{
	}

	void InputSystem::update(float dt)
	{
		ZoneScopedN("InputSystem::update");

		static bool app_focused_last_frame = true;

		const UserInput& input = get_current_frame_input();

		vec3 current_input = vec3(
			(input.keyboard['D'].down ? 1.0f : 0.0f) + (input.keyboard['A'].down ? -1.f : 0.0f),
			(input.keyboard['E'].down ? 1.0f : 0.0f) + (input.keyboard['Q'].down ? -1.f : 0.0f),
			(input.keyboard['W'].down ? -1.0f : 0.0f) + (input.keyboard['S'].down ? 1.f : 0.0f)
		);

		for (auto [handle, transform, sender] : world->group(components_group<TransformComponent, InputSenderComponent>).each())
		{
			sender.frame_input = input;
			sender.last_input = sender.current_input;
			sender.current_input = current_input;

			sender.notify_input();
		}

		app_focused_last_frame = ImGui::IsMousePosValid();
	}

}