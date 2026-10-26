#include "physics/physical_animation/ragdoll_drive_compiler.h"
#include "physics/physical_animation/physical_animation_component.h"
#include "physics/core/physics.h"
#include "physics/body_component.h"
#include "physics/joint.h"
#include "physics/physx_api.h"

#include <algorithm>

namespace era_engine::physics
{
    static constexpr float TWO_PI = 2.0f * M_PI;

    static physx::PxRigidDynamic* get_limb_rigid_dynamic(Entity limb)
    {
        const DynamicBodyComponent* body_component = limb.get_component_if_exists<DynamicBodyComponent>();
        return body_component != nullptr ? body_component->get_rigid_dynamic() : nullptr;
    }

    CompiledDriveValues CompiledDriveValues::lerp(const CompiledDriveValues& from, const CompiledDriveValues& to, float t)
    {
        CompiledDriveValues result;
        result.stiffness = ::lerp(from.stiffness, to.stiffness, t);
        result.damping = ::lerp(from.damping, to.damping, t);
        result.force_limit = ::lerp(from.force_limit, to.force_limit, t);
        result.accelerated = to.accelerated;
        return result;
    }

    CompiledLimbDrive CompiledLimbDrive::lerp(const CompiledLimbDrive& from, const CompiledLimbDrive& to, float t)
    {
        CompiledLimbDrive result;

        result.angular = CompiledDriveValues::lerp(from.angular, to.angular, t);
        result.linear = CompiledDriveValues::lerp(from.linear, to.linear, t);

        result.angular_frequency = ::lerp(from.angular_frequency, to.angular_frequency, t);
        result.angular_damping_ratio = ::lerp(from.angular_damping_ratio, to.angular_damping_ratio, t);
        result.torque_stiffness = ::lerp(from.torque_stiffness, to.torque_stiffness, t);
        result.max_torque = ::lerp(from.max_torque, to.max_torque, t);
        result.saturation_angle = ::lerp(from.saturation_angle, to.saturation_angle, t);

        result.linear_frequency = ::lerp(from.linear_frequency, to.linear_frequency, t);
        result.linear_damping_ratio = ::lerp(from.linear_damping_ratio, to.linear_damping_ratio, t);
        result.max_force = ::lerp(from.max_force, to.max_force, t);
        result.saturation_distance = ::lerp(from.saturation_distance, to.saturation_distance, t);

        return result;
    }

    LimbMassProperties RagdollDriveCompiler::compute_limb_mass_properties(const PhysicalAnimationLimbComponent* limb_component)
    {
        using namespace physx;

        LimbMassProperties result;

        ASSERT(limb_component != nullptr);

        const PxRigidDynamic* rigid_dynamic = get_limb_rigid_dynamic(limb_component->get_entity());
        if (rigid_dynamic == nullptr)
        {
            return result;
        }

        // Drive anchor in the limb actor frame (joint frame of actor1 of the drive joint).
        PxVec3 anchor = PxVec3(0.0f);
        if (const D6JointComponent* drive_joint = dynamic_cast<const D6JointComponent*>(limb_component->drive_joint_component.get()))
        {
            anchor = create_PxVec3(drive_joint->get_second_local_frame().position);
        }

        PhysicsEngine::execute_read([&]()
            {
                const float mass = rigid_dynamic->getMass();
                const PxTransform cmass_pose = rigid_dynamic->getCMassLocalPose();
                const PxVec3 inertia = rigid_dynamic->getMassSpaceInertiaTensor();

                if (mass <= 0.0f || inertia.x <= 0.0f || inertia.y <= 0.0f || inertia.z <= 0.0f)
                {
                    return;
                }

                // Lever arm from the centre of mass to the anchor, in the actor frame.
                const PxVec3 lever = anchor - cmass_pose.p;

                float best_ratio = 0.0f;
                float effective_mass_sum = 0.0f;

                for (uint32 axis = 0; axis < 3; ++axis)
                {
                    PxVec3 unit_axis = PxVec3(0.0f);
                    unit_axis[axis] = 1.0f;

                    const PxVec3 n = cmass_pose.q.rotate(unit_axis);

                    const float inertia_cm = inertia[axis];
                    const float distance_sq = std::max(lever.magnitudeSquared() - lever.dot(n) * lever.dot(n), 0.0f);
                    const float inertia_pivot = inertia_cm + mass * distance_sq;

                    const float ratio = inertia_pivot / inertia_cm;
                    if (ratio > best_ratio)
                    {
                        best_ratio = ratio;
                        result.inertia_cm = inertia_cm;
                        result.inertia_pivot = inertia_pivot;
                    }

                    // Linear row along n at the anchor: 1 / m_eff = 1 / m + (r x n)^T * I^-1 * (r x n).
                    const PxVec3 angular_part = cmass_pose.q.rotateInv(lever.cross(n));
                    const float angular_response = angular_part.x * angular_part.x / inertia.x +
                                                   angular_part.y * angular_part.y / inertia.y +
                                                   angular_part.z * angular_part.z / inertia.z;

                    effective_mass_sum += 1.0f / (1.0f / mass + angular_response);
                }

                result.mass = mass;
                result.center_of_mass = create_vec3(cmass_pose.p);
                result.linear_effective_mass = effective_mass_sum / 3.0f;
                result.valid = true;
            });

        return result;
    }

    CompiledLimbDrive RagdollDriveCompiler::compile_limb_drive(const MotorDriveSettings& settings, const LimbMassProperties& mass_properties, float strength)
    {
        CompiledLimbDrive result;

        if (!mass_properties.valid)
        {
            return result;
        }

        // Strength scales stiffness.
        const float sqrt_strength_coeff = std::sqrt(strength);

        // Angular drive.
        {
            const AngularDriveSettings& angular = settings.angular;

            const float omega = TWO_PI * angular.frequency;
            const float torque_stiffness = mass_properties.inertia_pivot * omega * omega;
            const float inertia_ratio = mass_properties.get_inertia_ratio();

            result.angular.stiffness = torque_stiffness / mass_properties.inertia_cm * strength;
            result.angular.damping = angular.damping_ratio * 2.0f * omega * inertia_ratio * sqrt_strength_coeff;

            result.torque_stiffness = torque_stiffness * strength;
            result.max_torque = mass_properties.inertia_pivot * angular.max_acceleration * strength;
            result.saturation_angle = result.torque_stiffness > 0.0f ? result.max_torque / result.torque_stiffness : 0.0f;

            // Slerp rows use half-length axes (PxD6Joint slerp Jacobian): the solver applies 0.5 * limit as torque per axis.
            result.angular.force_limit = 2.0f * result.max_torque;
            result.angular.accelerated = true;

            result.angular_frequency = omega * sqrt_strength_coeff / TWO_PI;
            result.angular_damping_ratio = angular.damping_ratio;
        }

        // Linear X/Y/Z drives at the anchor, designed for the limb mass.
        {
            const LinearDriveSettings& linear = settings.linear;

            const float omega = TWO_PI * linear.frequency;
            const float mass_ratio = mass_properties.get_linear_mass_ratio();

            result.linear.stiffness = omega * omega * mass_ratio * strength;
            result.linear.damping = linear.damping_ratio * 2.0f * omega * mass_ratio * sqrt_strength_coeff;

            // Force at the anchor is m_eff * k * x = m * w^2 * x.
            result.max_force = mass_properties.mass * linear.max_acceleration * strength;
            result.saturation_distance = omega > 0.0f ? linear.max_acceleration / (omega * omega) : 0.0f;
            result.linear.force_limit = result.max_force;
            result.linear.accelerated = true;

            result.linear_frequency = omega * sqrt_strength_coeff / TWO_PI;
            result.linear_damping_ratio = linear.damping_ratio;
        }

        return result;
    }

    float RagdollDriveCompiler::compute_hit_strength(const ContactResponseSettings& settings, float velocity_change)
    {
        const vec2& range = settings.velocity_change_range;
        if (velocity_change <= range.x)
        {
            return 1.0f;
        }

        const float t = range.y > range.x ? clamp((velocity_change - range.x) / (range.y - range.x), 0.0f, 1.0f) : 1.0f;
        return ::lerp(1.0f, settings.min_strength, t);
    }

    float RagdollDriveCompiler::get_contact_softness(const ContactResponseSettings& settings, float strength)
    {
        return settings.min_strength < 1.0f ? clamp((1.0f - strength) / (1.0f - settings.min_strength), 0.0f, 1.0f) : 0.0f;
    }

    CompiledLimbDrive RagdollDriveCompiler::apply_contact_strength(const CompiledLimbDrive& drive, const ContactResponseSettings& settings, float strength)
    {
        const float clamped_strength = clamp(strength, 0.0f, 1.0f);
        const float sqrt_strength = std::sqrt(clamped_strength);
        const float softness = get_contact_softness(settings, clamped_strength);

        auto apply = [&](CompiledDriveValues& values, float damping_ratio)
        {
            const float hit_damping_ratio = ::lerp(damping_ratio, settings.hit_damping_ratio, softness);
            const float damping_ratio_scale = damping_ratio > 0.0f ? hit_damping_ratio / damping_ratio : 1.0f;

            values.stiffness *= clamped_strength;
            values.damping *= sqrt_strength * damping_ratio_scale;
            values.force_limit *= clamped_strength;
        };

        CompiledLimbDrive result = drive;
        apply(result.angular, drive.angular_damping_ratio);
        apply(result.linear, drive.linear_damping_ratio);
        return result;
    }
}
