#include "physics/physical_animation/ragdoll_profiles_asset.h"

#include <core/log.h>

#include <ecs/serialization/reflection_serializer.h>

#include <rttr/registration>

#include <sstream>

namespace era_engine::physics
{
    static constexpr uint32 RAGDOLL_PROFILES_ASSET_VERSION = 2;

    struct RagdollProfilesData final
    {
        uint32 version = 0;
        RagdollProfileSelectionSettings selection_settings;
        std::vector<RagdollProfile> profiles;
    };

    RTTR_REGISTRATION
    {
        using namespace rttr;

        registration::class_<RagdollProfileSelectionSettings>("RagdollProfileSelectionSettings")
            .property("running_speed", &RagdollProfileSelectionSettings::running_speed)
            .property("sprint_speed", &RagdollProfileSelectionSettings::sprint_speed)
            .property("hysteresis", &RagdollProfileSelectionSettings::hysteresis)
            .property("transition_time", &RagdollProfileSelectionSettings::transition_time);

        registration::class_<RagdollProfilesData>("RagdollProfilesData")
            .property("version", &RagdollProfilesData::version)
            .property("selection_settings", &RagdollProfilesData::selection_settings)
            .property("profiles", &RagdollProfilesData::profiles);
    }

    bool RagdollProfilesAsset::serialize(std::ostream& os) const
    {
        RagdollProfilesData data;
        data.version = RAGDOLL_PROFILES_ASSET_VERSION;
        data.selection_settings = selection_settings;

        for (const ref<RagdollProfile>& profile : profiles)
        {
            if (profile != nullptr)
            {
                data.profiles.push_back(*profile);
            }
        }

        os << ReflectionSerializer::serialize(data);
        return os.good();
    }

    bool RagdollProfilesAsset::deserialize(std::istream& is)
    {
        std::stringstream text;
        text << is.rdbuf();

        RagdollProfilesData data;
        if (!ReflectionSerializer::deserialize(text.str(), data))
        {
            return false;
        }

        if (data.version != RAGDOLL_PROFILES_ASSET_VERSION)
        {
            LOG_ERROR("Ragdoll profiles> Unsupported version %u (expected %u).", data.version, RAGDOLL_PROFILES_ASSET_VERSION);
            return false;
        }

        selection_settings = data.selection_settings;

        profiles = {};
        for (const RagdollProfile& profile : data.profiles)
        {
            const uint32 index = static_cast<uint32>(profile.type);
            if (index < PROFILES_COUNT)
            {
                profiles[index] = make_ref<RagdollProfile>(profile);
            }
        }

        ++revision;
        return true;
    }

    std::string RagdollProfilesAsset::get_asset_type_impl()
    {
        return ".eragprof";
    }

    ref<RagdollProfile> RagdollProfilesAsset::get_profile(RagdollProfileType type) const
    {
        const uint32 index = static_cast<uint32>(type);
        if (index >= PROFILES_COUNT)
        {
            return nullptr;
        }
        return profiles[index];
    }

    static ContactResponseSettings make_contact_response(float min_strength,
        vec2 velocity_change_range,
        float chain_falloff_coeff,
        float parent_follow_factor,
        float contact_recovery_time,
        float hit_damping_ratio)
    {
        ContactResponseSettings settings;
        settings.min_strength = min_strength;
        settings.velocity_change_range = velocity_change_range;
        settings.chain_falloff_coeff = chain_falloff_coeff;
        settings.parent_follow_factor = parent_follow_factor;
        settings.contact_recovery_time = contact_recovery_time;
        settings.hit_damping_ratio = hit_damping_ratio;
        return settings;
    }

    static PhysicalLimbDetails make_driven_limb(float frequency,
        float damping_ratio,
        float linear_frequency,
        float angular_max_acceleration,
        float linear_max_acceleration,
        const ContactResponseSettings& contact_response)
    {
        PhysicalLimbDetails details;
        details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;

        MotorDriveSettings motor_drive;

        motor_drive.angular.frequency = frequency;
        motor_drive.angular.damping_ratio = damping_ratio;
        motor_drive.angular.max_acceleration = angular_max_acceleration;

        motor_drive.linear.frequency = linear_frequency;
        motor_drive.linear.damping_ratio = 1.0f;
        motor_drive.linear.max_acceleration = linear_max_acceleration;

        details.motor_drive = motor_drive;
        details.contact_response = contact_response;

        return details;
    }

    static ref<RagdollProfile> make_driven_profile(RagdollProfileType type, float frequency_scale, float damping_scale, float acceleration_scale)
    {
        ref<RagdollProfile> profile = make_ref<RagdollProfile>();
        profile->type = type;

        const float f = frequency_scale;
        const float z = damping_scale;
        const float a = acceleration_scale;

        ContactResponseSettings body_contact = make_contact_response(0.65f, vec2(0.6f, 4.0f), 0.3f, 0.5f, 0.6f, 1.0f);
        body_contact.touch_strength = 0.75f;
        ContactResponseSettings core_contact = make_contact_response(0.9f, vec2(1.0f, 6.0f), 0.3f, 0.5f, 0.3f, 1.0f);
        core_contact.touch_strength = 0.95f;
        const ContactResponseSettings neck_contact = make_contact_response(0.3f, vec2(0.5f, 3.0f), 0.6f, 1.0f, 0.6f, 1.0f);
        const ContactResponseSettings head_contact = make_contact_response(0.3f, vec2(0.5f, 3.0f), 0.6f, 0.8f, 0.6f, 1.0f);
        const ContactResponseSettings arm_contact = make_contact_response(0.4f, vec2(0.5f, 4.0f), 0.5f, 1.0f, 0.6f, 1.0f);
        const ContactResponseSettings forearm_contact = make_contact_response(0.25f, vec2(1.0f, 5.0f), 0.5f, 1.0f, 0.5f, 0.9f);
        const ContactResponseSettings hand_contact = make_contact_response(0.2f, vec2(1.5f, 6.0f), 0.5f, 1.0f, 0.4f, 0.8f);
        const ContactResponseSettings leg_contact = make_contact_response(0.6f, vec2(0.5f, 3.0f), 0.25f, 0.5f, 0.0f, 1.0f);
        const ContactResponseSettings calf_contact = make_contact_response(0.6f, vec2(0.5f, 3.0f), 0.25f, 0.5f, 0.5f, 1.0f);
        ContactResponseSettings foot_contact = make_contact_response(0.7f, vec2(1.0f, 5.0f), 0.25f, 0.4f, 0.5f, 1.0f);
        foot_contact.respond_to_own_hits = false;

        profile->body_lower_limb_details = make_driven_limb(8.0f * f, 1.0f * z, 6.5f * f, 350.0f * a, 45.0f * a, core_contact);
        profile->body_middle_limb_details = make_driven_limb(7.5f * f, 1.0f * z, 4.5f * f, 250.0f * a, 30.0f * a, core_contact);
        profile->body_upper_limb_details = make_driven_limb(7.5f * f, 1.0f * z, 4.5f * f, 250.0f * a, 30.0f * a, body_contact);

        profile->neck_limb_details = make_driven_limb(5.5f * f, 0.6f * z, 4.0f * f, 250.0f * a, 20.0f * a, neck_contact);
        profile->head_limb_details = make_driven_limb(5.5f * f, 0.6f * z, 4.0f * f, 250.0f * a, 20.0f * a, head_contact);

        profile->clavicle_limb_details = make_driven_limb(10.0f * f, 0.65f * z, 4.0f * f, 300.0f * a, 20.0f * a, arm_contact);
        profile->arm_limb_details = make_driven_limb(6.8f * f, 0.6f * z, 4.0f * f, 450.0f * a, 40.0f * a, arm_contact);
        profile->forearm_limb_details = make_driven_limb(8.2f * f, 0.65f * z, 4.25f * f, 600.0f * a, 40.0f * a, forearm_contact);
        profile->hand_limb_details = make_driven_limb(10.0f * f, 0.55f * z, 4.25f * f, 600.0f * a, 40.0f * a, hand_contact);

        profile->leg_limb_details = make_driven_limb(5.0f * f, 0.5f * z, 4.5f * f, 450.0f * a, 40.0f * a, leg_contact);
        profile->calf_limb_details = make_driven_limb(4.5f * f, 0.55f * z, 4.5f * f, 600.0f * a, 40.0f * a, calf_contact);
        profile->foot_limb_details = make_driven_limb(6.5f * f, 0.65f * z, 5.0f * f, 600.0f * a, 40.0f * a, foot_contact);

        return profile;
    }

    static ref<RagdollProfile> make_drag_profile()
    {
        ref<RagdollProfile> profile = make_driven_profile(RagdollProfileType::DRAG, 1.0f, 1.0f, 1.0f);

        MotorDriveSettings weak_drive;
        weak_drive.angular.frequency = 1.5f;
        weak_drive.angular.damping_ratio = 1.0f;
        weak_drive.angular.max_acceleration = 50.0f;
        weak_drive.linear.frequency = 1.0f;
        weak_drive.linear.damping_ratio = 1.0f;
        weak_drive.linear.max_acceleration = 5.0f;

        DragForceDetails drag_force;
        drag_force.partial_velocity_drive = 0.4f;
        drag_force.partial_velocity_drive_limit = 8.0f;
        drag_force.partial_angular_drive = 0.5f;
        drag_force.partial_angular_drive_limit = 12.0f;

        for (uint32 limb_index = 0; limb_index < RagdollProfile::LIMB_TYPES_COUNT; ++limb_index)
        {
            const RagdollLimbType limb_type = static_cast<RagdollLimbType>(limb_index);
            if (limb_type == RagdollLimbType::BODY_LOWER || limb_type == RagdollLimbType::BODY_MIDDLE || limb_type == RagdollLimbType::BODY_UPPER)
            {
                continue;
            }

            PhysicalLimbDetails& details = profile->get_limb_details_by_type(limb_type);
            details.motor_drive = weak_drive;
            details.drag_force = drag_force;
        }

        return profile;
    }

    ref<RagdollProfilesAsset> RagdollProfilesAsset::create_default()
    {
        ref<RagdollProfilesAsset> asset = make_ref<RagdollProfilesAsset>();

        asset->profiles[static_cast<uint32>(RagdollProfileType::IDLE)] = make_driven_profile(RagdollProfileType::IDLE, 1.0f, 1.0f, 1.0f);
        asset->profiles[static_cast<uint32>(RagdollProfileType::RUNNING)] = make_driven_profile(RagdollProfileType::RUNNING, 1.07f, 0.85f, 1.5f);
        asset->profiles[static_cast<uint32>(RagdollProfileType::SPRINT)] = make_driven_profile(RagdollProfileType::SPRINT, 1.12f, 0.8f, 2.0f);

        ref<RagdollProfile> ragdoll_profile = make_ref<RagdollProfile>();
        ragdoll_profile->type = RagdollProfileType::RAGDOLL;
        ragdoll_profile->head_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
        ragdoll_profile->arm_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
        ragdoll_profile->body_upper_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
        ragdoll_profile->body_middle_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
        ragdoll_profile->forearm_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
        ragdoll_profile->hand_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
        ragdoll_profile->leg_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
        ragdoll_profile->calf_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
        ragdoll_profile->foot_limb_details.blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
        asset->profiles[static_cast<uint32>(RagdollProfileType::RAGDOLL)] = ragdoll_profile;

        asset->profiles[static_cast<uint32>(RagdollProfileType::DRAG)] = make_drag_profile();

        return asset;
    }
}
