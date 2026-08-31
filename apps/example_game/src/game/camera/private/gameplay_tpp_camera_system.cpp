#include "game/camera/gameplay_tpp_camera_system.h"
#include "game/movement/character_locomotion_component.h"
#include "game/camera/gameplay_tpp_camera_component.h"

#include <ecs/update_groups.h>
#include <ecs/base_components/base_components.h>

#include <rttr/policy.h>
#include <rttr/registration>

#include <core/ecs/camera_holder_component.h>
#include <core/ecs/input_receiver_component.h>
#include <core/cpu_profiling.h>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<GameplayTppCameraSystem>("GameplayTppCameraSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr)(policy::ctor::as_raw_ptr, metadata("Tag", std::string("game")))
			.method("update", &GameplayTppCameraSystem::update)(metadata("update_group", update_types::GAMEPLAY_NORMAL));
	}

	GameplayTppCameraSystem::GameplayTppCameraSystem(World* _world)
		: System(_world)
	{

	}

	GameplayTppCameraSystem::~GameplayTppCameraSystem()
	{
	}

	void GameplayTppCameraSystem::init()
	{
	}

	void GameplayTppCameraSystem::update(float dt)
	{
		ZoneScopedN("GameplayTppCameraSystem::render");

		for (auto [handle, camera_holder, receiver, gameplay_camera, transform] : world->group(components_group<CameraHolderComponent, InputReceiverComponent, GameplayTppCameraComponent, TransformComponent>).each())
		{
			const UserInput& user_input = receiver.get_frame_input();
			const vec3& input = receiver.get_current_input();

			const trs& world_transform = transform.get_world_transform();

			render_camera* camera = camera_holder.get_render_camera();

			if (camera_holder.get_camera_type() == CameraHolderComponent::USER_DEFINED)
			{
                GameplayTppCameraComponent::OrbitState& orbit = gameplay_camera.orbit;

                if (!orbit.initialized)
                {
                    orbit.initialized = true;
                    orbit.smoothed_position = world_transform.position +
                        vec3(0, orbit.height, -orbit.distance);
                }

                if (user_input.mouse.captured)
                {
                    float mouse_dx = user_input.mouse.raw_dx;
                    float mouse_dy = user_input.mouse.raw_dy;

                    orbit.yaw -= mouse_dx * gameplay_camera.camera_sensitivity;
                    orbit.pitch += mouse_dy * gameplay_camera.camera_sensitivity;

                    orbit.pitch = clamp(orbit.pitch, gameplay_camera.camera_min_pitch, gameplay_camera.camera_max_pitch);
                }

                if (user_input.mouse.scroll_delta != 0.0f)
                {
                    orbit.distance -= user_input.mouse.scroll_delta * 0.5f;
                    orbit.distance = clamp(orbit.distance, 2.0f, 10.0f);
                }

                float yaw_rad = deg2rad(orbit.yaw);
                float pitch_rad = deg2rad(orbit.pitch);

                vec3 camera_offset;
                camera_offset.x = orbit.distance * cos(pitch_rad) * sin(yaw_rad);
                camera_offset.y = orbit.distance * sin(pitch_rad);
                camera_offset.z = orbit.distance * cos(pitch_rad) * cos(yaw_rad);

                vec3 target_position = world_transform.position + vec3(0, orbit.height, 0);

                vec3 desired_camera_position = target_position + camera_offset;

                float smooth_factor = 1.0f - exp(-gameplay_camera.camera_smooth_speed * dt);
                orbit.smoothed_position = lerp(orbit.smoothed_position, desired_camera_position, smooth_factor);

                camera->position = orbit.smoothed_position;

                vec3 look_direction = normalize(target_position - camera->position);

                camera->rotation = look_at_quaternion(look_direction, vec3(0, 1, 0));
			}
		}
	}
}