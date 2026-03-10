#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include "physics/ragdolls/ragdoll_component.h"

#include <ecs/entity.h>

#include <core/math.h>

namespace era_engine::animation
{
    class SkeletonComponent;
}

namespace era_engine::physics
{
    class RagdollProfile;

    struct ERA_PHYSICS_API RagdollSkeletonStructure final
    {
        struct JointStructure final
        {
            uint32 joint_id = INVALID_JOINT;
            trs joint_object_space_transform;

            std::optional<trs> constraint_object_space_transform;
            std::optional<Entity> physical_limb;
            std::optional<Entity> kinematic_target;
        };

        JointStructure head_end_joint;
        JointStructure head_joint;
        JointStructure neck_joint;

        JointStructure thorax_joint;
        JointStructure abdomen_joint;
        JointStructure pelvis_joint;

        JointStructure left_clavicle_joint;
        JointStructure left_arm_joint;
        JointStructure left_forearm_joint;
        JointStructure left_hand_joint;
        JointStructure left_hand_end_joint;

        JointStructure right_clavicle_joint;
        JointStructure right_arm_joint;
        JointStructure right_forearm_joint;
        JointStructure right_hand_joint;
        JointStructure right_hand_end_joint;

        JointStructure left_leg_joint;
        JointStructure left_calf_joint;
        JointStructure left_foot_joint;
        JointStructure left_foot_end_joint;

        JointStructure right_leg_joint;
        JointStructure right_calf_joint;
        JointStructure right_foot_joint;
        JointStructure right_foot_end_joint;

        float distance_between_arms = 0.0f;
        float distance_between_hand_and_hand_end = 0.0f;

        float distance_between_foot_and_foot_end = 0.0f;
        float distance_between_foot_y_and_foot_end_y = 0.0f;
    };

    class ERA_PHYSICS_API PhysicalRagdollBuilder final
    {
    public:
        // Mutable data is runtime and will be used by builder, so you only need to set up non-mutable context fields before ragdoll creation.
        struct RuntimeContext final
        {
            World* world = nullptr;
            bool enable_physical_animation = true;

            const RagdollProfile* default_profile = nullptr;

            mutable Entity ragdoll;
            mutable RagdollSkeletonStructure skeleton_structure;
            mutable RagdollComponent* ragdoll_component = nullptr;
        };

        PhysicalRagdollBuilder() = delete;

        static void build_ragdoll(const RuntimeContext& ctx);

    private:
        static bool prepare_structure(const RuntimeContext& ctx);
        static bool build_bodies(const RuntimeContext& ctx);
        static bool build_constraint_joints(const RuntimeContext& ctx);

        static bool build_kinematics(const RuntimeContext& ctx); // Optional stage. Only if enable_physical_animation == true.
        static bool build_drive_joints(const RuntimeContext& ctx); // Optional stage. Only if enable_physical_animation == true.
    };
}