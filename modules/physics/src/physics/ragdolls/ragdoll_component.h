#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"

#include <core/math.h>

#include <animation/animation.h>

#include <ecs/component.h>
#include <ecs/observable_member.h>

namespace era_engine::physics
{
    struct ERA_PHYSICS_API RagdollJointIds
    {
        uint32_t head_end_idx = INVALID_JOINT;
        uint32_t head_idx = INVALID_JOINT;
        uint32_t neck_idx = INVALID_JOINT;
        uint32_t spine_03_idx = INVALID_JOINT;
        uint32_t spine_02_idx = INVALID_JOINT;
        uint32_t spine_01_idx = INVALID_JOINT;
        uint32_t pelvis_idx = INVALID_JOINT;

        uint32_t clavicle_l_idx = INVALID_JOINT;
        uint32_t clavicle_r_idx = INVALID_JOINT;

        uint32_t root_idx = INVALID_JOINT;
        uint32_t attachment_idx = INVALID_JOINT;

        uint32_t thigh_l_idx = INVALID_JOINT;
        uint32_t calf_l_idx = INVALID_JOINT;
        uint32_t foot_l_idx = INVALID_JOINT;
        uint32_t foot_end_l_idx = INVALID_JOINT;

        uint32_t thigh_r_idx = INVALID_JOINT;
        uint32_t calf_r_idx = INVALID_JOINT;
        uint32_t foot_r_idx = INVALID_JOINT;
        uint32_t foot_end_r_idx = INVALID_JOINT;

        uint32_t upperarm_l_idx = INVALID_JOINT;
        uint32_t lowerarm_l_idx = INVALID_JOINT;
        uint32_t hand_l_idx = INVALID_JOINT;
        uint32_t hand_end_l_idx = INVALID_JOINT;

        uint32_t upperarm_r_idx = INVALID_JOINT;
        uint32_t lowerarm_r_idx = INVALID_JOINT;
        uint32_t hand_r_idx = INVALID_JOINT;
        uint32_t hand_end_r_idx = INVALID_JOINT;
    };

    class ERA_PHYSICS_API RagdollSettings final
    {
    public:
        float head_mass_percentage = 0.0826f;
        float body_upper_mass_percentage = 0.204f;
        float body_lower_mass_percentage = 0.204f;
        float arm_mass_percentage = 0.07f;
        float forearm_mass_percentage = 0.0467f;
        float hand_mass_percentage = 0.015f;
        float up_leg_mass_percentage = 0.085f;
        float leg_mass_percentage = 0.0475f;
        float foot_mass_percentage = 0.024f;

        float max_head_contact_impulse = 220.0f;
        float max_hand_contact_impulse = 60.0f;
        float max_forearm_contact_impulse = 60.0f;
        float max_arm_contact_impulse = 100.0f;
        float max_body_contact_impulse = 200.0f;
        float max_up_leg_contact_impulse = 150.0f;
        float max_leg_contact_impulse = 150.0f;
        float max_foot_contact_impulse = 40.0f;

        float arm_radius = 0.081f;
        float forearm_radius = 0.063f;
        float hand_width = 0.08f;
        float hand_height = 0.04f;
        float up_leg_radius = 0.09f;
        float leg_radius = 0.06f;
        float foot_width = 0.1f;
        float neck_radius = 0.07f;
        float neck_half_height = 0.03f;
        float head_radius = 0.1f;
        float head_half_height = 0.04f;

        vec3 head_joint_adjastment = vec3::zero;
        vec3 head_end_joint_adjastment = vec3::zero;
        vec3 neck_joint_adjastment = vec3::zero;
        vec3 thorax_joint_adjastment = vec3::zero;
        vec3 abdomen_joint_adjastment = vec3::zero;
        vec3 pelvis_joint_adjastment = vec3::zero;

        float upper_body_height_modifier = 1.0f;
        float middle_body_height_modifier = 1.0f;
        float lower_body_height_modifier = 1.0f;

        float upper_body_radius_modifier = 1.0f;
        float middle_body_radius_modifier = 1.0f;
        float lower_body_radius_modifier = 1.0f;
    };

    enum class RagdollLimbType : uint8
    {
        BODY_LOWER = 0,
        BODY_MIDDLE,
        BODY_UPPER,
        HEAD,
        NECK,
        CLAVICLE,
        ARM,
        FOREARM,
        HAND,
        LEG,
        CALF,
        FOOT
    };

	class ERA_PHYSICS_API RagdollLimbComponent : public Component
	{
	public:
		RagdollLimbComponent() = default;
		RagdollLimbComponent(ref<Entity::EcsData> _data, uint32 _joint_id = INVALID_JOINT);

		~RagdollLimbComponent() override;

		uint32 joint_id = INVALID_JOINT;

        /* Prev frame physics body local space transfrom. */
        trs prev_physics_pose = trs::identity;

        /* Physics body local space transfrom. */
        trs physics_pose = trs::identity;

        ObservableMember<bool> simulated = false;

        EntityPtr joint_entity_ptr;

        RagdollLimbType type = RagdollLimbType::BODY_LOWER;

        ERA_VIRTUAL_REFLECT(Component)
	};

	class ERA_PHYSICS_API RagdollComponent : public Component
	{
	public:
		RagdollComponent() = default;
		RagdollComponent(ref<Entity::EcsData> _data);
		~RagdollComponent() override;

		ObservableMember<bool> simulated = false;

        bool loaded = false;

        RagdollSettings settings;
        RagdollJointIds joint_init_ids;

        float mass = 100.0f; // Set before creation.

        float elapsed_blend_time = 0.0f;
        bool reached_physics_pose = false;

        float blend_factor = 0.5f;

        uint32 root_joint_id = 0;

		std::vector<EntityPtr> limbs;

        ERA_VIRTUAL_REFLECT(Component)

    protected:
        std::unordered_map<uint32, EntityPtr> simulated_joints;
        std::unordered_map<uint32, trs> local_joint_poses;
        std::set<uint32> simulated_joints_set;

        friend class RagdollSystem;
        friend class PhysicalAnimationSystem;
	};
}