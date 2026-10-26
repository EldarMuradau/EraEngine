#pragma once

#include "physics_api.h"

#include "physics/physical_animation/ragdoll_profile.h"

#include <core/math.h>

namespace era_engine::physics
{
    class PhysicalAnimationLimbComponent;

    struct ERA_PHYSICS_API LimbMassProperties final
    {
        bool valid = false;

        float mass = 0.0f;

        // Inertia along the principal axis with the largest I_pivot / I_cm (swing axis).
        float inertia_cm = 0.0f;
        float inertia_pivot = 0.0f;

        vec3 center_of_mass = vec3::zero;

        // Mean effective mass of the linear drive rows at the anchor.
        float linear_effective_mass = 0.0f;

        float get_inertia_ratio() const { return valid ? inertia_pivot / inertia_cm : 0.0f; }
        float get_linear_mass_ratio() const { return valid ? mass / linear_effective_mass : 0.0f; }
    };

    struct ERA_PHYSICS_API CompiledDriveValues final
    {
        static CompiledDriveValues lerp(const CompiledDriveValues& from, const CompiledDriveValues& to, float t);

        float stiffness = 0.0f;
        float damping = 0.0f;
        float force_limit = 0.0f;
        bool accelerated = true;
    };

    struct ERA_PHYSICS_API CompiledLimbDrive final
    {
        static CompiledLimbDrive lerp(const CompiledLimbDrive& from, const CompiledLimbDrive& to, float t);

        CompiledDriveValues angular;
        CompiledDriveValues linear;

        float angular_frequency = 0.0f;
        float angular_damping_ratio = 0.0f;
        float torque_stiffness = 0.0f; // About the joint, N*m/rad.
        float max_torque = 0.0f;       // N*m.
        float saturation_angle = 0.0f; // rad.

        float linear_frequency = 0.0f;
        float linear_damping_ratio = 0.0f;
        float max_force = 0.0f;
        float saturation_distance = 0.0f;
    };

    // Acceleration drives (PxD6JointDriveFlag::eACCELERATION): PhysX divides the spring by the effective mass of the
    // drive row, computed from the two actors of the drive joint only. The kinematic target contributes nothing, so
    // the row sees the limb as a free body: angular rows use I_cm, linear rows use m_eff. Then alpha = k * theta for a free limb, w^2 = k.
    //
    // The limb is not free: it is held at its joint by the parent constraint, so it rotates about the joint with
    // I_pivot = I_cm + m * d^2 and the drive acceleration drops by I_cm / I_pivot. The compiler compensates with a
    // dimensionless ratio that depends on the shape and the joint position only:
    //
    //     Angular: k = w^2 * (I_pivot / I_cm),   c = 2 * zeta * w * (I_pivot / I_cm)
    //     Linear:  k = w^2 * (m / m_eff),        c = 2 * zeta * w * (m / m_eff)
    //              m / m_eff = 1 + m * (r x n)^T I^-1 (r x n), the anchor is held by the chain, so the drive moves the limb mass.
    //     w = 2 * pi * f. E.g. a capsule driven at its end: I_pivot / I_cm = 4, k = 4 * w^2.
    //
    // Only force limits depend on mass:
    //     max torque = I_pivot * max_angular_acceleration, max force = m * max_linear_acceleration.
    class ERA_PHYSICS_API RagdollDriveCompiler final
    {
    public:
        RagdollDriveCompiler() = delete;

        static LimbMassProperties compute_limb_mass_properties(const PhysicalAnimationLimbComponent* limb_component);

        static CompiledLimbDrive compile_limb_drive(const MotorDriveSettings& settings, const LimbMassProperties& mass_properties, float strength);

        static float compute_hit_strength(const ContactResponseSettings& settings, float velocity_change);

        // 0 at full strength, 1 at min_strength.
        static float get_contact_softness(const ContactResponseSettings& settings, float strength);

        // Stiffness and limits scale by strength, damping by sqrt(strength) with the ratio moved towards hit_damping_ratio.
        static CompiledLimbDrive apply_contact_strength(const CompiledLimbDrive& drive, const ContactResponseSettings& settings, float strength);
    };
}
