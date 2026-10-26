#pragma once

#include "physics_api.h"
#include "physics/physx_api.h"
#include "physics/ragdolls/ragdoll_component.h"
#include "physics/physical_animation/ragdoll_profile.h"
#include "physics/physical_animation/ragdoll_profiles_asset.h"
#include "physics/physical_animation/ragdoll_drive_compiler.h"
#include "physics/physical_animation/states/base_simulation_state.h"
#include "physics/physical_animation/limb_states/base_limb_state.h"

#include <core/math.h>

#include <ecs/component.h>
#include <ecs/observable_member.h>

namespace era_engine::physics
{
    enum class RagdollLimbSide : uint8
    {
        CENTER = 0,
        LEFT,
        RIGHT
    };

    struct ERA_PHYSICS_API LimbDeviation final
    {
        float angle = 0.0f;
        float distance = 0.0f;

        // Normalized values are relative to the drive saturation.
        float normalized_angle = 0.0f;
        float normalized_distance = 0.0f;
    };

    class ERA_PHYSICS_API PhysicsLimbChain final
    {
    public:
        PhysicsLimbChain() = default;

        bool has_any_colliding_limb() const;

        bool has_any_dismembered_limb() const;

    public:
        std::vector<EntityPtr> connected_limbs;
    };

    class ERA_PHYSICS_API PhysicalAnimationLimbComponent : public RagdollLimbComponent
    {
    public:
        PhysicalAnimationLimbComponent() = default;
        PhysicalAnimationLimbComponent(ref<Entity::EcsData> _data, uint32 _joint_id = INVALID_JOINT);
        ~PhysicalAnimationLimbComponent() override;

        std::shared_ptr<BaseLimbState> get_current_state() const;

        PhysicalLimbStateType get_current_state_type() const;

        void force_switch_state(PhysicalLimbStateType desired_state);

        void update_states(float dt, PhysicalLimbStateType desired_state);

        void apply_impulse(const vec3& impulse, const vec3& local_point = vec3::zero);

    public:
        struct CollisionData
        {
            constexpr static float MAX_COLLISION_TIME = 1.0f;
            constexpr static float MAX_FREQUENT_COLLISION_TIME = 0.8f;

            bool was_in_collision = false;
            bool is_colliding = false;
            bool touching_dynamic = false;
            float collision_time = 0.0f;

            vec3 impulse = vec3::zero;

            void reset();
        };

        CollisionData collision;

        // Drive target in ragdoll space.
        trs target_pose = trs::identity;

        // Raw animation pose in ragdoll space.
        trs animation_pose = trs::identity;

        LimbMassProperties mass_properties;

        // Drive values compiled from the current profile, blended on profile changes.
        CompiledLimbDrive drive;
        CompiledLimbDrive drive_transition_from;
        CompiledLimbDrive drive_transition_to;

        float contact_strength = 1.0f;

        LimbDeviation deviation;

        RagdollLimbSide side = RagdollLimbSide::CENTER;

        float transition_time = 0.6f;

        PhysicalLimbBlendType blend_type = PhysicalLimbBlendType::BLEND_WITH_PREV_POSE;
        PhysicalLimbBlendType prev_blend_type = PhysicalLimbBlendType::NONE;

        ComponentPtr drive_joint_component = nullptr;

        EntityPtr ragdoll_ptr;
        EntityPtr drive_joint_entity_ptr;

        ERA_VIRTUAL_REFLECT(RagdollLimbComponent)

    private:
        PhysicalLimbStateType current_state_type = PhysicalLimbStateType::KINEMATIC;

        std::unordered_map<PhysicalLimbStateType, std::shared_ptr<BaseLimbState>> simulation_states;

        friend class PhysicalAnimationSystem;
    };

    class ERA_PHYSICS_API PhysicalAnimationComponent : public RagdollComponent
    {
    public:
        PhysicalAnimationComponent() = default;
        PhysicalAnimationComponent(ref<Entity::EcsData> _data);
        ~PhysicalAnimationComponent() override;

        bool try_to_apply_ragdoll_profile(ref<RagdollProfile> new_profile, RagdollProfileStrengthType new_strength_type = RagdollProfileStrengthType::DEFAULT, bool force_reload = false);
        ref<RagdollProfile> get_ragdoll_profile() const;
        RagdollProfileStrengthType get_strength_type() const;

        void update_drives(float dt);

        // Animation poses arrive at frame rate, drive targets move at fixed rate. The two latest animation poses are kept
        // and sampled at the physics clock, so targets move smoothly whatever the frame rate is (one frame behind).
        void push_animation_pose(const animation::SkeletonPose& pose, float dt);
        bool sample_animation_pose(float dt, animation::SkeletonPose& out_pose);

        std::shared_ptr<BaseSimulationState> get_current_state() const;

        void reset_blends();

        SimulationStateType get_current_state_type() const;

        void force_switch_state(SimulationStateType desired_state);

        float get_profile_transition_time() const;

        bool is_in_transition() const;

        void update_profile_transition(float dt);

        bool is_in_idle() const;
        bool is_in_ragdoll() const;

        void force_set_ragdoll(bool enabled);

        void update_states(float dt, SimulationStateType desired_state);

        float get_strength_value_by_limb(RagdollLimbType limb_type) const;

        const PhysicsLimbChain* get_chain_by_joint_id(uint32 joint_id) const;

        const PhysicalAnimationLimbComponent* get_limb(RagdollLimbType type, RagdollLimbSide side = RagdollLimbSide::CENTER) const;
        const PhysicalAnimationLimbComponent* get_limb_by_joint_id(uint32 joint_id) const;

        LimbDeviation get_limb_deviation(RagdollLimbType type, RagdollLimbSide side = RagdollLimbSide::CENTER) const;
        LimbDeviation get_max_deviation() const;

    public:
        constexpr static float MAX_RAGDOLL_PROFILE_TRANSITION_TIME = 0.3f;

        bool use_fixed_pelvis_attachment = false;

        float blend_weight = 0.0f;

        float blend_in_time = 0.1f;
        float blend_out_time = 0.2f;

        float animation_blend_factor = 0.5f;

        trs prev_world_transform = trs::identity;

        vec3 velocity = vec3::zero;

        ref<PhysicsLimbChain> left_arm_chain;
        ref<PhysicsLimbChain> right_arm_chain;

        ref<PhysicsLimbChain> left_leg_chain;
        ref<PhysicsLimbChain> right_leg_chain;

        ref<PhysicsLimbChain> neck_chain;
        ref<PhysicsLimbChain> body_chain;

        EntityPtr attachment_body;

        float pelvis_attachment_stiffness = 0.0f;
        float pelvis_attachment_damping = 0.0f;

        ref<RagdollProfilesAsset> profiles_asset;

        ERA_VIRTUAL_REFLECT(RagdollComponent)

    private:
        void update_contact_strengths(float dt);

        ref<RagdollProfile> current_profile = nullptr;

        std::unordered_map<uint32, trs> local_joint_poses_for_target_calculation;
        std::unordered_map<uint32, trs> physics_follow_joint_poses;
        std::unordered_map<uint32, trs> animation_joint_poses;

        std::vector<int32> limb_parent_indices;

        SimulationStateType current_state_type = SimulationStateType::DISABLED;
        SkeletonUpdateType mesh_update_type = SkeletonUpdateType::TRANSFORM;

        float ragdoll_profile_transition_time = 0.0f;

        bool drives_dirty = true;
        bool drives_compiled = false;
        bool mass_properties_initialized = false;
        uint32 applied_asset_revision = UINT32_MAX;

        float drive_transition_elapsed = 0.0f;
        float drive_transition_duration = 0.0f;

        struct AnimationPoseSnapshot
        {
            animation::SkeletonPose pose;
            float time = 0.0f;
        };

        // [0] - previous frame, [1] - latest frame.
        AnimationPoseSnapshot animation_pose_snapshots[2];
        uint32 animation_pose_snapshots_count = 0;

        float animation_elapsed = 0.0f;
        float physics_elapsed = 0.0f;

        std::unordered_map<SimulationStateType, std::shared_ptr<BaseSimulationState>> simulation_states;

        RagdollProfileStrengthType strength_type = RagdollProfileStrengthType::DEFAULT;
        bool force_ragdoll_state = false;

        friend class PhysicalAnimationSystem;
        friend class RagdollDrivePoseSampler;
        friend class PhysicalAnimationBuilderUtils;
    };
}