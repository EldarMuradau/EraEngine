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

    class ERA_PHYSICS_API RagdollSettings
    {
    public:
        struct MassSettings
        {
            float head_mass_percentage = 0.0826f;
            float neck_mass_percentage = 0.0826f;
            float body_upper_mass_percentage = 0.204f;
            float body_middle_mass_percentage = 0.204f;
            float body_lower_mass_percentage = 0.204f;
            float clavicle_mass_percentage = 0.07f;
            float arm_mass_percentage = 0.07f;
            float forearm_mass_percentage = 0.0467f;
            float hand_mass_percentage = 0.015f;
            float leg_mass_percentage = 0.085f;
            float calf_mass_percentage = 0.0475f;
            float foot_mass_percentage = 0.024f;
        };

        struct ContactImpulseSettings
        {
            float max_head_contact_impulse = 200.0f;
            float max_neck_contact_impulse = 180.0f;
            float max_hand_contact_impulse = 60.0f;
            float max_forearm_contact_impulse = 60.0f;
            float max_arm_contact_impulse = 100.0f;
            float max_body_contact_impulse = 200.0f;
            float max_leg_contact_impulse = 150.0f;
            float max_calf_contact_impulse = 150.0f;
            float max_foot_contact_impulse = 40.0f;
        };

        struct BaseShapesSettings
        {
            float clavicle_radius = 0.081f;
            float arm_radius = 0.081f;
            float forearm_radius = 0.063f;
            float hand_x = 0.06f;
            float hand_y = 0.04f;
            float hand_z = 0.02f;
            float leg_radius = 0.09f;
            float calf_radius = 0.06f;
            float foot_radius = 0.1f;
            float neck_radius = 0.07f;
            float head_radius = 0.1f;
        };

        struct LocalShapePositionsSettings
        {
            vec3 head_joint_adjastment = vec3::zero;
            vec3 head_end_joint_adjastment = vec3::zero;
            vec3 neck_joint_adjastment = vec3::zero;

            vec3 thorax_joint_adjastment = vec3::zero;
            vec3 abdomen_joint_adjastment = vec3::zero;
            vec3 pelvis_joint_adjastment = vec3::zero;

            vec3 left_clavicle_joint_adjastment = vec3::zero;
            vec3 left_arm_joint_adjastment = vec3::zero;
            vec3 left_forearm_joint_adjastment = vec3::zero;
            vec3 left_hand_joint_adjastment = vec3::zero;

            vec3 right_clavicle_joint_adjastment = vec3::zero;
            vec3 right_arm_joint_adjastment = vec3::zero;
            vec3 right_forearm_joint_adjastment = vec3::zero;
            vec3 right_hand_joint_adjastment = vec3::zero;

            vec3 left_thigh_joint_adjastment = vec3::zero;
            vec3 left_calf_joint_adjastment = vec3::zero;
            vec3 left_foot_joint_adjastment = vec3::zero;

            vec3 right_thigh_joint_adjastment = vec3::zero;
            vec3 right_calf_joint_adjastment = vec3::zero;
            vec3 right_foot_joint_adjastment = vec3::zero;

            quat head_joint_spin = quat::identity;
            quat head_end_joint_spin = quat::identity;
            quat neck_joint_spin = quat::identity;

            quat thorax_joint_spin = quat::identity;
            quat abdomen_joint_spin = quat::identity;
            quat pelvis_joint_spin = quat::identity;

            quat left_clavicle_joint_spin = quat::identity;
            quat left_arm_joint_spin = quat::identity;
            quat left_forearm_joint_spin = quat::identity;
            quat left_hand_joint_spin = quat::identity;

            quat right_clavicle_joint_spin = quat::identity;
            quat right_arm_joint_spin = quat::identity;
            quat right_forearm_joint_spin = quat::identity;
            quat right_hand_joint_spin = quat::identity;

            quat left_thigh_joint_spin = quat::identity;
            quat left_calf_joint_spin = quat::identity;
            quat left_foot_joint_spin = quat::identity;

            quat right_thigh_joint_spin = quat::identity;
            quat right_calf_joint_spin = quat::identity;
            quat right_foot_joint_spin = quat::identity;
        };

        struct ObjectSpaceJointsSettings
        {
            vec3 head_joint_adjastment = vec3::zero;
            vec3 head_end_joint_adjastment = vec3::zero;
            vec3 neck_joint_adjastment = vec3::zero;

            vec3 thorax_joint_adjastment = vec3::zero;
            vec3 abdomen_joint_adjastment = vec3::zero;
            vec3 pelvis_joint_adjastment = vec3::zero;

            vec3 left_clavicle_joint_adjastment = vec3::zero;
            vec3 left_arm_joint_adjastment = vec3::zero;
            vec3 left_forearm_joint_adjastment = vec3::zero;
            vec3 left_hand_joint_adjastment = vec3::zero;

            vec3 right_clavicle_joint_adjastment = vec3::zero;
            vec3 right_arm_joint_adjastment = vec3::zero;
            vec3 right_forearm_joint_adjastment = vec3::zero;
            vec3 right_hand_joint_adjastment = vec3::zero;

            vec3 left_thigh_joint_adjastment = vec3::zero;
            vec3 left_calf_joint_adjastment = vec3::zero;
            vec3 left_foot_joint_adjastment = vec3::zero;
            vec3 left_foot_end_joint_adjastment = vec3::zero;

            vec3 right_thigh_joint_adjastment = vec3::zero;
            vec3 right_calf_joint_adjastment = vec3::zero;
            vec3 right_foot_joint_adjastment = vec3::zero;
            vec3 right_foot_end_joint_adjastment = vec3::zero;
        };

        struct ShapeScalerSettings
        {
            float upper_body_height_modifier = 1.0f;
            float middle_body_height_modifier = 1.0f;
            float lower_body_height_modifier = 1.0f;

            float upper_body_radius_modifier = 1.0f;
            float middle_body_radius_modifier = 1.0f;
            float lower_body_radius_modifier = 1.0f;

            float clavicle_height_modifier = 1.0f;
            float arm_height_modifier = 1.0f;
            float forearm_height_modifier = 1.0f;
        };

        MassSettings mass_settings;
        ContactImpulseSettings impulse_settings;
        BaseShapesSettings shapes_settings;
        LocalShapePositionsSettings local_shape_settings;
        ObjectSpaceJointsSettings object_space_settings;
        ShapeScalerSettings scaler_settings;
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