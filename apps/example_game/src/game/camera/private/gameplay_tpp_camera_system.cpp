#include "game/camera/gameplay_tpp_camera_system.h"
#include "game/movement/character_locomotion_component.h"
#include "game/camera/gameplay_tpp_camera_component.h"

#include <motion_matching/motion/motion_component.h>

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
            .method("update", &GameplayTppCameraSystem::update)(metadata("update_group", update_types::BEFORE_RENDER),
                metadata("After", std::vector<std::string>{"CameraSystem::update"}));
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
        ZoneScopedN("GameplayTppCameraSystem::update");

        for (auto [handle, camera_holder, receiver, gameplay_camera, transform, motion] 
            : world->group(components_group<CameraHolderComponent, 
				InputReceiverComponent,
				GameplayTppCameraComponent,
				TransformComponent,
				MotionComponent>).each())
		{
			if (camera_holder.get_camera_type() == CameraHolderComponent::USER_DEFINED)
            {
                const UserInput& user_input = receiver.get_frame_input();
                const vec3& input = receiver.get_current_input();

                const trs& world_transform = transform.get_world_transform();

                render_camera* camera = camera_holder.get_render_camera();

                GameplayTppCameraComponent::OrbitState& orbit = gameplay_camera.orbit;

                if (user_input.mouse.captured)
                {
                    const float mouse_dx = user_input.mouse.raw_dx;
                    const float mouse_dy = user_input.mouse.raw_dy;

                    orbit.yaw -= mouse_dx * gameplay_camera.camera_sensitivity;
                    orbit.pitch += mouse_dy * gameplay_camera.camera_sensitivity;

                    orbit.pitch = clamp(orbit.pitch, gameplay_camera.camera_min_pitch, gameplay_camera.camera_max_pitch);
                }

                if (user_input.mouse.scroll_delta != 0.0f)
                {
                    orbit.distance -= user_input.mouse.scroll_delta * 0.5f;
                    orbit.distance = clamp(orbit.distance, 2.0f, 10.0f);
                }

                const float yaw_rad = deg2rad(orbit.yaw);
                const float pitch_rad = deg2rad(orbit.pitch);

                vec3 camera_offset;
                camera_offset.x = orbit.distance * cos(pitch_rad) * sin(yaw_rad);
                camera_offset.y = orbit.distance * sin(pitch_rad);
                camera_offset.z = orbit.distance * cos(pitch_rad) * cos(yaw_rad);

                const vec3 target_position = world_transform.position + vec3(0, orbit.height, 0);
                const vec3 desired_camera_position = target_position + camera_offset;

                camera->position = desired_camera_position;

                vec3 look_delta_position = target_position - camera->position;
                const vec3 look_direction = normalize(look_delta_position);
                camera->rotation = look_at_quaternion(look_direction, vec3::up);

                camera->updateMatrices();

                look_delta_position.y = 0.0f;
                const vec3 look_yaw_direction = normalize(look_delta_position);

                motion.set_input_movement_rotation(look_at_quaternion(look_yaw_direction, vec3::up));
            }
        }
    }
}