#include "core/ecs/private/camera_system.h"
#include "core/log.h"
#include "core/cpu_profiling.h"
#include "core/ecs/input_receiver_component.h"
#include "core/ecs/camera_holder_component.h"

#include "rendering/ecs/renderer_holder_root_component.h"

#include "ecs/base_components/transform_component.h"
#include "ecs/update_groups.h"

#include <rttr/policy.h>
#include <rttr/registration>

#include <imgui/imgui.h>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<CameraSystem>("CameraSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("base")))
			.method("update", &CameraSystem::update)(metadata("update_group", update_types::BEFORE_RENDER));
	}

	CameraSystem::CameraSystem(World* _world)
		: System(_world)
	{
		renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();
		ASSERT(renderer_holder_rc != nullptr);
	}

	CameraSystem::~CameraSystem()
	{
	}

	void CameraSystem::init()
	{
	}

	void CameraSystem::update(float dt)
	{
		ZoneScopedN("CameraSystem::render");

		for (auto [handle, camera_holder, receiver, transform] : world->group(components_group<CameraHolderComponent, InputReceiverComponent, TransformComponent>).each())
		{
			const UserInput& user_input = receiver.get_frame_input();
			const vec3& input = receiver.get_current_input();

			render_camera* camera = camera_holder.get_render_camera();

			camera->setViewport(renderer_holder_rc->width, renderer_holder_rc->height);

			if (camera_holder.get_camera_type() == CameraHolderComponent::FREE_CAMERA ||
				(camera_holder.get_camera_type() == CameraHolderComponent::FREE_CAMERA_ON_HOLD && user_input.mouse.right.down))
			{
				const trs& world_transform = transform.get_world_transform();

				const float CAMERA_MOVEMENT_SPEED = 8.0f;
				const float CAMERA_SENSITIVITY = 4.0f;

				vec2 turn_angle(0.f, 0.f);
				turn_angle = vec2(-user_input.mouse.reldx, -user_input.mouse.reldy) * CAMERA_SENSITIVITY;

				quat& camera_rotation = camera->rotation;
				camera_rotation = quat(vec3(0.f, 1.f, 0.f), turn_angle.x) * camera_rotation;
				camera_rotation = camera_rotation * quat(vec3(1.f, 0.f, 0.f), turn_angle.y);

				camera->position += camera_rotation * input * dt * CAMERA_MOVEMENT_SPEED;

				transform.set_world_transform(trs{ camera->position, camera->rotation, world_transform.scale });

				camera->updateMatrices();

			}
			else if(camera_holder.get_camera_type() == CameraHolderComponent::ATTACHED_TO_TRS)
			{
				const trs& world_transform = transform.get_world_transform();

				camera->position = world_transform.position;
				camera->rotation = world_transform.rotation;

				camera->updateMatrices();
			}
		}

		for (auto [handle, camera_holder, transform] : world->group(components_group<CameraHolderComponent, TransformComponent>, components_group<InputReceiverComponent>).each())
		{
			render_camera* camera = camera_holder.get_render_camera();

			camera->setViewport(renderer_holder_rc->width, renderer_holder_rc->height);

			const trs& world_transform = transform.get_world_transform();

			camera->position = world_transform.position;
			camera->rotation = world_transform.rotation;

			camera->updateMatrices();
		}
	}

}