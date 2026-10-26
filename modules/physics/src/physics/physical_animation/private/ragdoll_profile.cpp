#include "physics/physical_animation/ragdoll_profile.h"

#include <ecs/serialization/value_adapters.h>

#include <rttr/registration>

namespace era_engine::physics
{
    RTTR_REGISTRATION
    {
        using namespace rttr;

        registration::enumeration<RagdollProfileType>("RagdollProfileType")(
            value("IDLE", RagdollProfileType::IDLE),
            value("RUNNING", RagdollProfileType::RUNNING),
            value("SPRINT", RagdollProfileType::SPRINT),
            value("CLIMBING", RagdollProfileType::CLIMBING),
            value("RAGDOLL", RagdollProfileType::RAGDOLL),
            value("DRAG", RagdollProfileType::DRAG));

        registration::enumeration<RagdollProfileStrengthType>("RagdollProfileStrengthType")(
            value("DEFAULT", RagdollProfileStrengthType::DEFAULT),
            value("SOFT", RagdollProfileStrengthType::SOFT),
            value("HARD", RagdollProfileStrengthType::HARD));

        registration::enumeration<PhysicalLimbBlendType>("PhysicalLimbBlendType")(
            value("NONE", PhysicalLimbBlendType::NONE),
            value("BLEND_WITH_PREV_POSE", PhysicalLimbBlendType::BLEND_WITH_PREV_POSE),
            value("BLEND_WITH_ANIMATION_POSE", PhysicalLimbBlendType::BLEND_WITH_ANIMATION_POSE),
            value("PURE_PHYSICS", PhysicalLimbBlendType::PURE_PHYSICS),
            value("PURE_ANIMATION", PhysicalLimbBlendType::PURE_ANIMATION));
        ValueAdapters::register_enum_converter<PhysicalLimbBlendType>();

        registration::class_<AngularDriveSettings>("AngularDriveSettings")
            .property("frequency", &AngularDriveSettings::frequency)
            .property("damping_ratio", &AngularDriveSettings::damping_ratio)
            .property("max_acceleration", &AngularDriveSettings::max_acceleration);

        registration::class_<LinearDriveSettings>("LinearDriveSettings")
            .property("frequency", &LinearDriveSettings::frequency)
            .property("damping_ratio", &LinearDriveSettings::damping_ratio)
            .property("max_acceleration", &LinearDriveSettings::max_acceleration);

        registration::class_<ContactResponseSettings>("ContactResponseSettings")
            .property("min_strength", &ContactResponseSettings::min_strength)
            .property("touch_strength", &ContactResponseSettings::touch_strength)
            .property("hit_damping_ratio", &ContactResponseSettings::hit_damping_ratio)
            .property("velocity_change_range", &ContactResponseSettings::velocity_change_range)
            .property("contact_recovery_time", &ContactResponseSettings::contact_recovery_time)
            .property("chain_falloff_coeff", &ContactResponseSettings::chain_falloff_coeff)
            .property("parent_follow_factor", &ContactResponseSettings::parent_follow_factor)
            .property("respond_to_own_hits", &ContactResponseSettings::respond_to_own_hits);
        ValueAdapters::register_optional<ContactResponseSettings>();

        registration::class_<MotorDriveSettings>("MotorDriveSettings")
            .property("angular", &MotorDriveSettings::angular)
            .property("linear", &MotorDriveSettings::linear);
        ValueAdapters::register_optional<MotorDriveSettings>();

        registration::class_<DragForceDetails>("DragForceDetails")
            .property("partial_angular_drive_limit", &DragForceDetails::partial_angular_drive_limit)
            .property("partial_angular_drive", &DragForceDetails::partial_angular_drive)
            .property("partial_velocity_drive_limit", &DragForceDetails::partial_velocity_drive_limit)
            .property("partial_velocity_drive", &DragForceDetails::partial_velocity_drive)
            .property("acceleration_limit", &DragForceDetails::acceleration_limit)
            .property("acceleration_gain", &DragForceDetails::acceleration_gain);
        ValueAdapters::register_optional<DragForceDetails>();

        registration::class_<LimbStrengthDetails>("LimbStrengthDetails")
            .property("default_strength_coeff", &LimbStrengthDetails::default_strength_coeff)
            .property("soft_strength_coeff", &LimbStrengthDetails::soft_strength_coeff)
            .property("hard_strength_coeff", &LimbStrengthDetails::hard_strength_coeff);

        registration::class_<PhysicalLimbDetails>("PhysicalLimbDetails")
            .property("blend_type", &PhysicalLimbDetails::blend_type)
            .property("strength_details", &PhysicalLimbDetails::strength_details)
            .property("motor_drive", &PhysicalLimbDetails::motor_drive)
            .property("drag_force", &PhysicalLimbDetails::drag_force)
            .property("contact_response", &PhysicalLimbDetails::contact_response);

        registration::class_<RagdollProfile>("RagdollProfile")
            .property("type", &RagdollProfile::type)
            .property("head_limb_details", &RagdollProfile::head_limb_details)
            .property("neck_limb_details", &RagdollProfile::neck_limb_details)
            .property("body_upper_limb_details", &RagdollProfile::body_upper_limb_details)
            .property("body_middle_limb_details", &RagdollProfile::body_middle_limb_details)
            .property("body_lower_limb_details", &RagdollProfile::body_lower_limb_details)
            .property("clavicle_limb_details", &RagdollProfile::clavicle_limb_details)
            .property("arm_limb_details", &RagdollProfile::arm_limb_details)
            .property("forearm_limb_details", &RagdollProfile::forearm_limb_details)
            .property("hand_limb_details", &RagdollProfile::hand_limb_details)
            .property("leg_limb_details", &RagdollProfile::leg_limb_details)
            .property("calf_limb_details", &RagdollProfile::calf_limb_details)
            .property("foot_limb_details", &RagdollProfile::foot_limb_details);
    }

    const PhysicalLimbDetails& RagdollProfile::get_limb_details_by_type(RagdollLimbType limb_type) const
    {
        return const_cast<RagdollProfile*>(this)->get_limb_details_by_type(limb_type);
    }

    PhysicalLimbDetails& RagdollProfile::get_limb_details_by_type(RagdollLimbType limb_type)
    {
        switch (limb_type)
        {
            case RagdollLimbType::BODY_LOWER:
                return body_lower_limb_details;

            case RagdollLimbType::BODY_MIDDLE:
                return body_middle_limb_details;

            case RagdollLimbType::BODY_UPPER:
                return body_upper_limb_details;

            case RagdollLimbType::HEAD:
                return head_limb_details;

            case RagdollLimbType::NECK:
                return neck_limb_details;

            case RagdollLimbType::CLAVICLE:
                return clavicle_limb_details;

            case RagdollLimbType::ARM:
                return arm_limb_details;

            case RagdollLimbType::FOREARM:
                return forearm_limb_details;

            case RagdollLimbType::HAND:
                return hand_limb_details;

            case RagdollLimbType::LEG:
                return leg_limb_details;

            case RagdollLimbType::CALF:
                return calf_limb_details;

            case RagdollLimbType::FOOT:
                return foot_limb_details;
        }
        return body_lower_limb_details;
    }

    float RagdollProfile::get_strength(RagdollLimbType limb_type, RagdollProfileStrengthType strength_type) const
    {
        const LimbStrengthDetails& strength_details = get_limb_details_by_type(limb_type).strength_details;

        if (strength_type == RagdollProfileStrengthType::SOFT)
        {
            return strength_details.soft_strength_coeff;
        }
        else if (strength_type == RagdollProfileStrengthType::HARD)
        {
            return strength_details.hard_strength_coeff;
        }

        return strength_details.default_strength_coeff;
    }
}
