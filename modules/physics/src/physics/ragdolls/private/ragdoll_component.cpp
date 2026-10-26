#include "physics/ragdolls/ragdoll_component.h"

#include <ecs/serialization/component_registry.h>
#include <ecs/serialization/serialization_meta.h>

#include <rttr/registration>

namespace era_engine::physics
{

    RTTR_REGISTRATION
    {
        using namespace rttr;

        registration::enumeration<RagdollLimbType>("RagdollLimbType")(
            value("BODY_LOWER", RagdollLimbType::BODY_LOWER),
            value("BODY_MIDDLE", RagdollLimbType::BODY_MIDDLE),
            value("BODY_UPPER", RagdollLimbType::BODY_UPPER),
            value("HEAD", RagdollLimbType::HEAD),
            value("NECK", RagdollLimbType::NECK),
            value("CLAVICLE", RagdollLimbType::CLAVICLE),
            value("ARM", RagdollLimbType::ARM),
            value("FOREARM", RagdollLimbType::FOREARM),
            value("HAND", RagdollLimbType::HAND),
            value("LEG", RagdollLimbType::LEG),
            value("CALF", RagdollLimbType::CALF),
            value("FOOT", RagdollLimbType::FOOT));

        registration::class_<RagdollJointIds>("RagdollJointIds")
            .property("head_end_idx", &RagdollJointIds::head_end_idx)
            .property("head_idx", &RagdollJointIds::head_idx)
            .property("neck_idx", &RagdollJointIds::neck_idx)
            .property("spine_03_idx", &RagdollJointIds::spine_03_idx)
            .property("spine_02_idx", &RagdollJointIds::spine_02_idx)
            .property("spine_01_idx", &RagdollJointIds::spine_01_idx)
            .property("pelvis_idx", &RagdollJointIds::pelvis_idx)
            .property("clavicle_l_idx", &RagdollJointIds::clavicle_l_idx)
            .property("clavicle_r_idx", &RagdollJointIds::clavicle_r_idx)
            .property("root_idx", &RagdollJointIds::root_idx)
            .property("attachment_idx", &RagdollJointIds::attachment_idx)
            .property("thigh_l_idx", &RagdollJointIds::thigh_l_idx)
            .property("calf_l_idx", &RagdollJointIds::calf_l_idx)
            .property("foot_l_idx", &RagdollJointIds::foot_l_idx)
            .property("foot_end_l_idx", &RagdollJointIds::foot_end_l_idx)
            .property("thigh_r_idx", &RagdollJointIds::thigh_r_idx)
            .property("calf_r_idx", &RagdollJointIds::calf_r_idx)
            .property("foot_r_idx", &RagdollJointIds::foot_r_idx)
            .property("foot_end_r_idx", &RagdollJointIds::foot_end_r_idx)
            .property("upperarm_l_idx", &RagdollJointIds::upperarm_l_idx)
            .property("lowerarm_l_idx", &RagdollJointIds::lowerarm_l_idx)
            .property("hand_l_idx", &RagdollJointIds::hand_l_idx)
            .property("hand_end_l_idx", &RagdollJointIds::hand_end_l_idx)
            .property("upperarm_r_idx", &RagdollJointIds::upperarm_r_idx)
            .property("lowerarm_r_idx", &RagdollJointIds::lowerarm_r_idx)
            .property("hand_r_idx", &RagdollJointIds::hand_r_idx)
            .property("hand_end_r_idx", &RagdollJointIds::hand_end_r_idx);

        registration::class_<RagdollSettings::MassSettings>("RagdollMassSettings")
            .property("head_mass_percentage", &RagdollSettings::MassSettings::head_mass_percentage)
            .property("neck_mass_percentage", &RagdollSettings::MassSettings::neck_mass_percentage)
            .property("body_upper_mass_percentage", &RagdollSettings::MassSettings::body_upper_mass_percentage)
            .property("body_middle_mass_percentage", &RagdollSettings::MassSettings::body_middle_mass_percentage)
            .property("body_lower_mass_percentage", &RagdollSettings::MassSettings::body_lower_mass_percentage)
            .property("clavicle_mass_percentage", &RagdollSettings::MassSettings::clavicle_mass_percentage)
            .property("arm_mass_percentage", &RagdollSettings::MassSettings::arm_mass_percentage)
            .property("forearm_mass_percentage", &RagdollSettings::MassSettings::forearm_mass_percentage)
            .property("hand_mass_percentage", &RagdollSettings::MassSettings::hand_mass_percentage)
            .property("leg_mass_percentage", &RagdollSettings::MassSettings::leg_mass_percentage)
            .property("calf_mass_percentage", &RagdollSettings::MassSettings::calf_mass_percentage)
            .property("foot_mass_percentage", &RagdollSettings::MassSettings::foot_mass_percentage);

        registration::class_<RagdollSettings::ContactImpulseSettings>("RagdollContactImpulseSettings")
            .property("max_velocity_change", &RagdollSettings::ContactImpulseSettings::max_velocity_change);

        registration::class_<RagdollSettings::ContactMaterialSettings>("RagdollContactMaterialSettings")
            .property("restitution", &RagdollSettings::ContactMaterialSettings::restitution)
            .property("static_friction", &RagdollSettings::ContactMaterialSettings::static_friction)
            .property("dynamic_friction", &RagdollSettings::ContactMaterialSettings::dynamic_friction)
            .property("foot_static_friction", &RagdollSettings::ContactMaterialSettings::foot_static_friction)
            .property("foot_dynamic_friction", &RagdollSettings::ContactMaterialSettings::foot_dynamic_friction)
            .property("hand_static_friction", &RagdollSettings::ContactMaterialSettings::hand_static_friction)
            .property("hand_dynamic_friction", &RagdollSettings::ContactMaterialSettings::hand_dynamic_friction);

        registration::class_<RagdollSettings::BaseShapesSettings>("RagdollBaseShapesSettings")
            .property("clavicle_radius", &RagdollSettings::BaseShapesSettings::clavicle_radius)
            .property("arm_radius", &RagdollSettings::BaseShapesSettings::arm_radius)
            .property("forearm_radius", &RagdollSettings::BaseShapesSettings::forearm_radius)
            .property("hand_radius", &RagdollSettings::BaseShapesSettings::hand_radius)
            .property("leg_radius", &RagdollSettings::BaseShapesSettings::leg_radius)
            .property("calf_radius", &RagdollSettings::BaseShapesSettings::calf_radius)
            .property("foot_radius", &RagdollSettings::BaseShapesSettings::foot_radius)
            .property("neck_radius", &RagdollSettings::BaseShapesSettings::neck_radius)
            .property("head_radius", &RagdollSettings::BaseShapesSettings::head_radius);

        registration::class_<RagdollSettings::LocalShapePositionsSettings>("RagdollLocalShapePositionsSettings")
            .property("head_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::head_joint_adjastment)
            .property("head_end_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::head_end_joint_adjastment)
            .property("neck_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::neck_joint_adjastment)
            .property("thorax_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::thorax_joint_adjastment)
            .property("abdomen_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::abdomen_joint_adjastment)
            .property("pelvis_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::pelvis_joint_adjastment)
            .property("left_clavicle_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::left_clavicle_joint_adjastment)
            .property("left_arm_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::left_arm_joint_adjastment)
            .property("left_forearm_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::left_forearm_joint_adjastment)
            .property("left_hand_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::left_hand_joint_adjastment)
            .property("right_clavicle_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::right_clavicle_joint_adjastment)
            .property("right_arm_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::right_arm_joint_adjastment)
            .property("right_forearm_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::right_forearm_joint_adjastment)
            .property("right_hand_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::right_hand_joint_adjastment)
            .property("left_thigh_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::left_thigh_joint_adjastment)
            .property("left_calf_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::left_calf_joint_adjastment)
            .property("left_foot_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::left_foot_joint_adjastment)
            .property("right_thigh_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::right_thigh_joint_adjastment)
            .property("right_calf_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::right_calf_joint_adjastment)
            .property("right_foot_joint_adjastment", &RagdollSettings::LocalShapePositionsSettings::right_foot_joint_adjastment)
            .property("head_joint_spin", &RagdollSettings::LocalShapePositionsSettings::head_joint_spin)
            .property("head_end_joint_spin", &RagdollSettings::LocalShapePositionsSettings::head_end_joint_spin)
            .property("neck_joint_spin", &RagdollSettings::LocalShapePositionsSettings::neck_joint_spin)
            .property("thorax_joint_spin", &RagdollSettings::LocalShapePositionsSettings::thorax_joint_spin)
            .property("abdomen_joint_spin", &RagdollSettings::LocalShapePositionsSettings::abdomen_joint_spin)
            .property("pelvis_joint_spin", &RagdollSettings::LocalShapePositionsSettings::pelvis_joint_spin)
            .property("left_clavicle_joint_spin", &RagdollSettings::LocalShapePositionsSettings::left_clavicle_joint_spin)
            .property("left_arm_joint_spin", &RagdollSettings::LocalShapePositionsSettings::left_arm_joint_spin)
            .property("left_forearm_joint_spin", &RagdollSettings::LocalShapePositionsSettings::left_forearm_joint_spin)
            .property("left_hand_joint_spin", &RagdollSettings::LocalShapePositionsSettings::left_hand_joint_spin)
            .property("right_clavicle_joint_spin", &RagdollSettings::LocalShapePositionsSettings::right_clavicle_joint_spin)
            .property("right_arm_joint_spin", &RagdollSettings::LocalShapePositionsSettings::right_arm_joint_spin)
            .property("right_forearm_joint_spin", &RagdollSettings::LocalShapePositionsSettings::right_forearm_joint_spin)
            .property("right_hand_joint_spin", &RagdollSettings::LocalShapePositionsSettings::right_hand_joint_spin)
            .property("left_thigh_joint_spin", &RagdollSettings::LocalShapePositionsSettings::left_thigh_joint_spin)
            .property("left_calf_joint_spin", &RagdollSettings::LocalShapePositionsSettings::left_calf_joint_spin)
            .property("left_foot_joint_spin", &RagdollSettings::LocalShapePositionsSettings::left_foot_joint_spin)
            .property("right_thigh_joint_spin", &RagdollSettings::LocalShapePositionsSettings::right_thigh_joint_spin)
            .property("right_calf_joint_spin", &RagdollSettings::LocalShapePositionsSettings::right_calf_joint_spin)
            .property("right_foot_joint_spin", &RagdollSettings::LocalShapePositionsSettings::right_foot_joint_spin);

        registration::class_<RagdollSettings::ObjectSpaceJointsSettings>("RagdollObjectSpaceJointsSettings")
            .property("head_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::head_joint_adjastment)
            .property("head_end_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::head_end_joint_adjastment)
            .property("neck_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::neck_joint_adjastment)
            .property("thorax_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::thorax_joint_adjastment)
            .property("abdomen_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::abdomen_joint_adjastment)
            .property("pelvis_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::pelvis_joint_adjastment)
            .property("left_clavicle_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::left_clavicle_joint_adjastment)
            .property("left_arm_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::left_arm_joint_adjastment)
            .property("left_forearm_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::left_forearm_joint_adjastment)
            .property("left_hand_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::left_hand_joint_adjastment)
            .property("right_clavicle_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::right_clavicle_joint_adjastment)
            .property("right_arm_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::right_arm_joint_adjastment)
            .property("right_forearm_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::right_forearm_joint_adjastment)
            .property("right_hand_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::right_hand_joint_adjastment)
            .property("left_thigh_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::left_thigh_joint_adjastment)
            .property("left_calf_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::left_calf_joint_adjastment)
            .property("left_foot_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::left_foot_joint_adjastment)
            .property("left_foot_end_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::left_foot_end_joint_adjastment)
            .property("right_thigh_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::right_thigh_joint_adjastment)
            .property("right_calf_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::right_calf_joint_adjastment)
            .property("right_foot_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::right_foot_joint_adjastment)
            .property("right_foot_end_joint_adjastment", &RagdollSettings::ObjectSpaceJointsSettings::right_foot_end_joint_adjastment);

        registration::class_<RagdollSettings::ShapeScalerSettings>("RagdollShapeScalerSettings")
            .property("upper_body_height_modifier", &RagdollSettings::ShapeScalerSettings::upper_body_height_modifier)
            .property("middle_body_height_modifier", &RagdollSettings::ShapeScalerSettings::middle_body_height_modifier)
            .property("lower_body_height_modifier", &RagdollSettings::ShapeScalerSettings::lower_body_height_modifier)
            .property("upper_body_radius_modifier", &RagdollSettings::ShapeScalerSettings::upper_body_radius_modifier)
            .property("middle_body_radius_modifier", &RagdollSettings::ShapeScalerSettings::middle_body_radius_modifier)
            .property("lower_body_radius_modifier", &RagdollSettings::ShapeScalerSettings::lower_body_radius_modifier)
            .property("neck_height_modifier", &RagdollSettings::ShapeScalerSettings::neck_height_modifier)
            .property("clavicle_height_modifier", &RagdollSettings::ShapeScalerSettings::clavicle_height_modifier)
            .property("arm_height_modifier", &RagdollSettings::ShapeScalerSettings::arm_height_modifier)
            .property("forearm_height_modifier", &RagdollSettings::ShapeScalerSettings::forearm_height_modifier);

        registration::class_<RagdollSettings::BodySettings>("RagdollBodySettings")
            .property("max_depenetration_velocity", &RagdollSettings::BodySettings::max_depenetration_velocity)
            .property("linear_damping", &RagdollSettings::BodySettings::linear_damping)
            .property("angular_damping", &RagdollSettings::BodySettings::angular_damping)
            .property("max_linear_velocity", &RagdollSettings::BodySettings::max_linear_velocity)
            .property("max_angular_velocity", &RagdollSettings::BodySettings::max_angular_velocity)
            .property("sleep_threshold", &RagdollSettings::BodySettings::sleep_threshold);

        registration::class_<RagdollSettings::ConstraintLimitSettings>("RagdollConstraintLimitSettings")
            .property("frequency", &RagdollSettings::ConstraintLimitSettings::frequency)
            .property("damping_ratio", &RagdollSettings::ConstraintLimitSettings::damping_ratio);

        registration::class_<RagdollSettings::PelvisAttachmentSettings>("RagdollPelvisAttachmentSettings")
            .property("frequency", &RagdollSettings::PelvisAttachmentSettings::frequency)
            .property("damping_ratio", &RagdollSettings::PelvisAttachmentSettings::damping_ratio)
            .property("max_distance", &RagdollSettings::PelvisAttachmentSettings::max_distance);

        registration::class_<RagdollSettings>("RagdollSettings")
            .property("mass_settings", &RagdollSettings::mass_settings)
            .property("body_settings", &RagdollSettings::body_settings)
            .property("constraint_limit_settings", &RagdollSettings::constraint_limit_settings)
            .property("pelvis_attachment_settings", &RagdollSettings::pelvis_attachment_settings)
            .property("impulse_settings", &RagdollSettings::impulse_settings)
            .property("material_settings", &RagdollSettings::material_settings)
            .property("shapes_settings", &RagdollSettings::shapes_settings)
            .property("local_shape_settings", &RagdollSettings::local_shape_settings)
            .property("object_space_settings", &RagdollSettings::object_space_settings)
            .property("scaler_settings", &RagdollSettings::scaler_settings);

        registration::class_<RagdollComponent>("RagdollComponent")
            .constructor<ref<Entity::EcsData>>()
            .property("simulated", &RagdollComponent::simulated)
            .property("settings", &RagdollComponent::settings)
            .property("joint_init_ids", &RagdollComponent::joint_init_ids)
            .property("mass", &RagdollComponent::mass)
            .property("root_joint_id", &RagdollComponent::root_joint_id);
        ComponentRegistry::register_component<RagdollComponent>();

        registration::class_<RagdollLimbComponent>("RagdollLimbComponent")(rttr::metadata(SerializationMeta::NOT_SERIALIZABLE_CLASS, true))
            .constructor<>();
        ComponentRegistry::register_component<RagdollLimbComponent>();
    }

    RagdollLimbComponent::RagdollLimbComponent(ref<Entity::EcsData> _data, uint32 _joint_id /*= INVALID_JOINT*/)
        : Component(_data), joint_id(_joint_id)
    {
        simulated.set_component(ComponentPtr(this));
    }

    RagdollLimbComponent::~RagdollLimbComponent()
    {
    }

    RagdollComponent::RagdollComponent(ref<Entity::EcsData> _data)
        : Component(_data)
    {
        simulated.set_component(ComponentPtr(this));
    }

    RagdollComponent::~RagdollComponent()
    {
    }

}