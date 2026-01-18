#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include "physics/ragdolls/ragdoll_component.h"

#include <core/math.h>

#include <animation/animation.h>

#include <ecs/component.h>

namespace era_engine::physics
{
    enum class SkeletalMeshBlendType : uint8
    {
        ROTATION = 0,
        TRANSLATION,
        TRANSFORM
    };

    enum class PhysicalLimbBlendType : uint8
    {
        NONE = 0,
        BLEND_WITH_PREV_POSE = 1 << 0,
        BLEND_WITH_ANIMATION_POSE = 1 << 1,
        PURE_PHYSICS = 1 << 2,
        PURE_ANIMATION = 1 << 3
    };
    DEFINE_BITWISE_OPERATORS_FOR_ENUM(PhysicalLimbBlendType);

    enum class MotorDriveType : uint8
    {
        NONE = 0,
        VELOCITY = 1 << 0,
        TRANSFORM = 1 << 1,
        ALL = VELOCITY | TRANSFORM
    };
    DEFINE_BITWISE_OPERATORS_FOR_ENUM(MotorDriveType);

    class ERA_PHYSICS_API MotorDriveDetails final
    {
    public:
        MotorDriveDetails() = default;
        MotorDriveDetails(const MotorDriveDetails& other) = default;
        MotorDriveDetails(MotorDriveDetails&& other) = default;

        MotorDriveDetails& operator=(const MotorDriveDetails& other) = default;

        float angular_drive_stiffness = 100.0f;
        float linear_drive_stiffness = 100.0f;

        vec2 angular_range = vec2(deg2rad(3.0f), deg2rad(30.0f));
        vec2 angular_damping_range = vec2(60.0f, 10.0f);

        vec2 linear_range = vec2(0.05f, 0.5f);
        vec2 linear_damping_range = vec2(80.0f, 10.0f);

        float max_force = std::numeric_limits<float>::max();

        float velocity_drive_modifier = 0.25f;

        bool accelerated = true;
        bool enable_slerp_drive = true;
        MotorDriveType drive_type = MotorDriveType::TRANSFORM;
    };

    class ERA_PHYSICS_API DragForceDetails final
    {
    public:
        float partial_angular_drive_limit = 25.0f;
        float partial_angular_drive = 0.5f;

        float partial_velocity_drive_limit = 100.0f;
        float partial_velocity_drive = 0.5f;

        float acceleration_limit = 0.2f;
        float acceleration_gain = 0.02f;
    };

    struct ERA_PHYSICS_API LimbStrengthDetails final
    {
        float default_strength_coeff = 1.0f;

        float soft_strength_coeff = 0.5f;
        float hard_strength_coeff = 3.0f;
    };

    class ERA_PHYSICS_API PhysicalLimbDetails final
    {
    public:
        std::optional<MotorDriveDetails> motor_drive;
        std::optional<DragForceDetails> drag_force;

        LimbStrengthDetails strength_details;

        PhysicalLimbBlendType blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
    };

    enum class RagdollProfileType : uint8
    {
        IDLE = 0,
        RUNNING,
        SPRINT,
        CLIMBING,
        RAGDOLL,

        COUNT,

        DEFAULT = IDLE
    };

    enum class RagdollProfileStrengthType : uint8
    {
        DEFAULT = 0,
        SOFT,
        HARD
    };

    class ERA_PHYSICS_API RagdollProfile final
    {
    public:
        const PhysicalLimbDetails& get_limb_details_by_type(RagdollLimbType limb_type) const;

        PhysicalLimbDetails head_limb_details;
        PhysicalLimbDetails neck_limb_details;

        PhysicalLimbDetails body_upper_limb_details;
        PhysicalLimbDetails body_middle_limb_details;
        PhysicalLimbDetails body_lower_limb_details;

        PhysicalLimbDetails clavicle_limb_details;
        PhysicalLimbDetails arm_limb_details;
        PhysicalLimbDetails forearm_limb_details;
        PhysicalLimbDetails hand_limb_details;

        PhysicalLimbDetails leg_limb_details;
        PhysicalLimbDetails calf_limb_details;
        PhysicalLimbDetails foot_limb_details;

        RagdollProfileType type = RagdollProfileType::IDLE;
    };

    struct ERA_PHYSICS_API RagdollStrengthConfig
    {
        static constexpr float ANGULAR_DAMPING_MODIFIER = 1.5f;
        static constexpr float ANGULAR_STIFFNESS_MODIFIER = 1.5f;

        static constexpr float LINEAR_DAMPING_MODIFIER = 1.0f;
        static constexpr float LINEAR_STIFFNESS_MODIFIER = 1.0f;
    };
}