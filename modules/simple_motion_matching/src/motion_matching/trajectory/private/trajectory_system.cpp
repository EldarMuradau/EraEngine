#include "motion_matching/trajectory/trajectory_system.h"
#include "motion_matching/motion_matching_database.h"
#include "motion_matching/motion_matching_component.h"
#include "motion_matching/motion/spring_motion_utils.h"
#include "motion_matching/trajectory/trajectory_component.h"
#include "motion_matching/motion/motion_utils.h"
#include "motion_matching/motion/motion_component.h"

#include "motion_matching/common.h"

#include "rendering/ecs/renderer_holder_root_component.h"
#include "rendering/debug_visualization.h"

#include "core/cpu_profiling.h"
#include "core/string.h"
#include "core/ecs/input_receiver_component.h"
#include "core/debug/debug_var.h"

#include "engine/engine.h"

#include "ecs/update_groups.h"
#include "ecs/rendering/mesh_component.h"
#include "ecs/base_components/transform_component.h"

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
    static DebugVar<bool> draw_rotation = DebugVar<bool>("motion_matching.debug_draw.draw_rotation", false);
    static DebugVar<bool> draw_trajectories = DebugVar<bool>("motion_matching.debug_draw.draw_trajectories", false);

    // Predict what the desired velocity will be in the 
    // future. Here we need to use the future trajectory 
    // rotation as well as predicted future camera 
    // position to find an accurate desired velocity in 
    // the world space
    static void trajectory_desired_velocities_predict(
        slice1d<vec3> desired_velocities,
        const vec3& desired_velocity,
        const vec3& input,
        const vec3& raw_input,
        float fwrd_speed,
        float side_speed,
        float back_speed)
    {
        desired_velocities(0) = desired_velocity;

        for (int i = 1; i < desired_velocities.size; i++)
        {
            desired_velocities(i) = MotionUtils::desired_velocity_update(
                input,
                raw_input,
                fwrd_speed,
                side_speed,
                back_speed);
        }
    }

    static void trajectory_positions_predict(
        slice1d<vec3> positions,
        slice1d<vec3> velocities,
        slice1d<vec3> accelerations,
        const slice1d<float>& time_offsets,
        const vec3& position,
        const vec3& velocity,
        const vec3& acceleration,
        const slice1d<vec3>& desired_velocities,
        float halflife)
    {
        positions(0) = position;
        velocities(0) = velocity;
        accelerations(0) = acceleration;

        for (int i = 1; i < positions.size; i++)
        {
            positions(i) = positions(i - 1);
            velocities(i) = velocities(i - 1);
            accelerations(i) = accelerations(i - 1);

            MotionUtils::simulation_positions_update(
                positions(i),
                velocities(i),
                accelerations(i),
                desired_velocities(i),
                halflife,
                time_offsets(i));
        }
    }

    // Predict desired rotations given the estimated future 
    // camera rotation and other parameters
    static void trajectory_desired_rotations_predict(
        slice1d<quat> desired_rotations,
        const slice1d<vec3>& desired_velocities,
        const quat& desired_rotation,
        const vec3& input,
        bool desired_strafe,
        float strafe_direction = 0.0f)
    {
        desired_rotations(0) = desired_rotation;

        for (int i = 1; i < desired_rotations.size; i++)
        {
            desired_rotations(i) = MotionUtils::desired_rotation_update(
                desired_rotations(i - 1),
                input,
                strafe_direction,
                desired_strafe,
                desired_velocities(i));
        }
    }

    static void trajectory_rotations_predict(
        slice1d<quat> rotations,
        slice1d<vec3> angular_velocities,
        const quat& rotation,
        const vec3& angular_velocity,
        const slice1d<quat>& desired_rotations,
        float halflife,
        const slice1d<float>& time_offsets)
    {
        rotations.set(rotation);
        angular_velocities.set(angular_velocity);

        for (int i = 1; i < rotations.size; i++)
        {
            MotionUtils::simulation_rotations_update(
                rotations(i),
                angular_velocities(i),
                desired_rotations(i),
                halflife,
                time_offsets(i));
        }
    }

    RTTR_REGISTRATION
    {
        using namespace rttr;

        registration::class_<TrajectoryMotionSystem>("TrajectoryMotionSystem")
            .constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("motion_matching")))
            .method("update", &TrajectoryMotionSystem::update)(metadata("update_group", update_types::GAMEPLAY_BEFORE_PHYSICS),
                metadata("After", std::vector<std::string>{"MotionSystem::update"}))
            .method("debug_draw_update", &TrajectoryMotionSystem::debug_draw_update)(metadata("update_group", update_types::RENDER));
    }

    TrajectoryMotionSystem::TrajectoryMotionSystem(World* _world)
        : System(_world)
    {
        renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();
        ASSERT(renderer_holder_rc != nullptr);
    }

    TrajectoryMotionSystem::~TrajectoryMotionSystem()
    {
    }

    void TrajectoryMotionSystem::init()
    {
    }

    void TrajectoryMotionSystem::update(float dt)
    {
        using namespace animation;

        for (auto [handle, transform_component, reciever_component, trajectory_component, motion_component] 
            : world->group(components_group<TransformComponent, InputReceiverComponent, TrajectoryComponent, MotionComponent>).each())
        {
            const trs& current_world_transform = transform_component.get_world_transform();

            // Get gamepad stick states
            const vec3 raw_input = noz(motion_component.get_desired_input());
            const vec3& input = motion_component.applied_input_direction;

            // Get if strafe is desired
            bool desired_strafe = reciever_component.get_frame_input().keyboard[key_ctrl].down;

            const float strafe_direction = 0.0f;

            // Get the desired simulation speeds based on the gait
            float simulation_fwrd_speed = lerpf(motion_component.run_fwrd_speed, motion_component.walk_fwrd_speed, motion_component.desired_gait);
            float simulation_side_speed = lerpf(motion_component.run_side_speed, motion_component.walk_side_speed, motion_component.desired_gait);
            float simulation_back_speed = lerpf(motion_component.run_back_speed, motion_component.walk_back_speed, motion_component.desired_gait);

            trajectory_desired_velocities_predict(
                trajectory_component.trajectory_desired_velocities,
                motion_component.velocity,
                input,
                raw_input,
                simulation_fwrd_speed,
                simulation_side_speed,
                simulation_back_speed);

            trajectory_desired_rotations_predict(
                trajectory_component.trajectory_desired_rotations,
                trajectory_component.trajectory_desired_velocities,
                motion_component.desired_rotation,
                input,
                desired_strafe,
                strafe_direction);

            trajectory_rotations_predict(
                trajectory_component.trajectory_rotations,
                trajectory_component.trajectory_angular_velocities,
                current_world_transform.rotation,
                motion_component.angular_velocity,
                trajectory_component.trajectory_desired_rotations,
                motion_component.rotation_halflife,
                trajectory_component.time_offsets);

            trajectory_positions_predict(
                trajectory_component.trajectory_positions,
                trajectory_component.trajectory_velocities,
                trajectory_component.trajectory_accelerations,
                trajectory_component.time_offsets,
                current_world_transform.position,
                motion_component.velocity,
                motion_component.acceleration,
                trajectory_component.trajectory_desired_velocities,
                motion_component.velocity_halflife);
        }
    }

    void TrajectoryMotionSystem::debug_draw_update(float dt)
    {
        if (draw_trajectories || draw_rotation)
        {
            for (auto [handle, transform_component, trajectory_component]
                : world->group(components_group<TransformComponent, TrajectoryComponent>).each())
            {
                if (draw_trajectories)
                {
                    draw_trajectory(
                        trajectory_component.trajectory_positions,
                        trajectory_component.trajectory_rotations,
                        vec4(1.0f, 0.0f, 0.0f, 1.0f));
                }

                if (draw_rotation)
                {
                    const trs& world_transform = transform_component.get_world_transform();
                    vec3 dir = world_transform.rotation * vec3::forward;
                    renderLine(world_transform.position, world_transform.position + 0.6f * dir, vec4(1.0f, 1.0f, 0.0f, 1.0f), renderer_holder_rc->ldrRenderPass);
                }
            }
        }
    }

    void TrajectoryMotionSystem::draw_trajectory(const slice1d<vec3>& trajectory_positions, 
        const slice1d<quat>& trajectory_rotations, 
        const vec4& color)
    {
        for (int i = 1; i < trajectory_positions.size; i++)
        {
            vec3 point = trajectory_positions(i);
            renderWireSphere(point, 0.05f, color, renderer_holder_rc->ldrRenderPass);

            vec3 dir = trajectory_rotations(i) * vec3(0.0f, 0.0f, 1.0f);
            renderLine(point, point + 0.6f * dir, color, renderer_holder_rc->ldrRenderPass);
            renderLine(trajectory_positions(i - 1), point, color, renderer_holder_rc->ldrRenderPass);
        }
    }

}