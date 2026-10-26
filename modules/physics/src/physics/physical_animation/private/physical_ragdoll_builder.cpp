#include "physics/physical_animation/physical_ragdoll_builder.h"

#include "physics/core/physics.h"
#include "physics/body_component.h"
#include "physics/shape_component.h"
#include "physics/core/physics_utils.h"
#include "physics/shape_utils.h"
#include "physics/physical_animation/physical_animation_component.h"
#include "physics/aggregate_holder_component.h"
#include "physics/joint.h"
#include "physics/physx_api.h"

#include <core/math.h>
#include <core/debug/debug_var.h>
#include <core/log.h>

#include <ecs/base_components/transform_component.h>
#include <ecs/world.h>

#include <algorithm>

#include <ecs/prefab/transient_component.h>

namespace era_engine::physics
{
    static Entity create_child_entity(
        bool is_physically_animated,
        Entity parent,
        RagdollComponent* ragdoll_component,
        std::string_view entity_name,
        const PhysicalLimbDetails* details,
        uint32 joint_id,
        RagdollLimbType type,
        PhysicsLimbChain* chain = nullptr)
    {
        Entity child = parent.get_world()->create_entity(entity_name.data());
        child.add_component<TransientComponent>();
        child.set_parent(parent.get_handle());

        if (is_physically_animated)
        {
            PhysicalAnimationLimbComponent* limb_component = child.add_component<PhysicalAnimationLimbComponent>();
            limb_component->joint_id = joint_id;
            limb_component->type = type;
            limb_component->blend_type = details->blend_type;

            limb_component->ragdoll_ptr = EntityPtr{parent};
        }
        else
        {
            RagdollLimbComponent* limb_component = child.add_component<RagdollLimbComponent>();
            limb_component->joint_id = joint_id;
            limb_component->type = type;
        }

        ragdoll_component->limbs.push_back(EntityPtr{child});

        if (chain != nullptr)
        {
            chain->connected_limbs.push_back(EntityPtr{child});
        }

        return child;
    }

    static DynamicBodyComponent* create_dynamic_body(
        bool is_physically_animated,
        Entity& entity,
        const float mass,
        const RagdollSettings& settings)
    {
        const RagdollSettings::BodySettings& body_settings = settings.body_settings;

        DynamicBodyComponent* dynamic_body_component = entity.add_component<DynamicBodyComponent>();
        dynamic_body_component->mass.get_for_write() = mass;
        dynamic_body_component->max_depenetration_velocity = body_settings.max_depenetration_velocity;
        dynamic_body_component->use_gravity.get_for_write() = !is_physically_animated;
        dynamic_body_component->simulated.get_for_write() = false;
        dynamic_body_component->enable_gyroscopic_forces.get_for_write() = true;
        dynamic_body_component->linear_damping.get_for_write() = body_settings.linear_damping;
        dynamic_body_component->angular_damping.get_for_write() = body_settings.angular_damping;
        dynamic_body_component->stabilization_threshold.get_for_write() = 0.0f;
        dynamic_body_component->sleep_threshold.get_for_write() = body_settings.sleep_threshold;
        dynamic_body_component->max_contact_impulse.get_for_write() = mass * settings.impulse_settings.max_velocity_change;
        dynamic_body_component->max_angular_velocity.get_for_write() = body_settings.max_angular_velocity;
        dynamic_body_component->max_linear_velocity.get_for_write() = body_settings.max_linear_velocity;
        dynamic_body_component->kinematic_motion_type.get_for_write() = KinematicMotionType::VELOCITY;

        const bool is_tgs = PhysicsEngine::get_physics_core()->get_descriptor().enable_tgs_solver;
        const bool is_gpu = PhysicsEngine::get_physics_core()->is_gpu();

        if (is_tgs)
        {
            dynamic_body_component->solver_velocity_iterations_count.get_for_write() = is_gpu ? 1 : 1;
            dynamic_body_component->solver_position_iterations_count.get_for_write() = is_gpu ? 24 : 10;
        }
        else
        {
            dynamic_body_component->solver_velocity_iterations_count.get_for_write() = is_gpu ? 8 : 6;
            dynamic_body_component->solver_position_iterations_count.get_for_write() = is_gpu ? 16 : 12;
        }

        return dynamic_body_component;
    }

    static Entity create_attachment_dynamic_body(
        Entity entity,
        std::string_view entity_name,
        const float mass)
    {
        ASSERT(entity.is_valid());

        Entity attachment = entity.get_world()->create_entity(entity_name.data());
        attachment.add_component<TransientComponent>();
        attachment.set_parent(entity.get_handle());

        DynamicBodyComponent* dynamic_body_component = attachment.add_component<DynamicBodyComponent>();
        dynamic_body_component->mass = mass;
        dynamic_body_component->use_gravity.get_for_write() = false;
        dynamic_body_component->simulated.get_for_write() = false;
        dynamic_body_component->kinematic.get_for_write() = true;
        dynamic_body_component->kinematic_motion_type.get_for_write() = KinematicMotionType::VELOCITY;

        SphereShapeComponent* shape_component = attachment.add_component<SphereShapeComponent>();
        shape_component->radius = 0.01f;
        shape_component->use_in_scene_queries = false;
        shape_component->collision_type = CollisionType::NONE;

        return attachment;
    }

    template <typename T>
    T* create_shape(Entity entity, const ref<PhysicsMaterial>& material, CollisionType collision_type = CollisionType::RAGDOLL)
    {
        T* shape_component = entity.add_component<T>();
        shape_component->collision_type = collision_type;
        shape_component->material = material;

        return shape_component;
    }

    static void create_d6_joint(
        bool is_physiclly_animated,
        const Entity& source,
        RagdollSkeletonStructure::JointStructure& e0_joint,
        RagdollSkeletonStructure::JointStructure& e1_joint,
        const trs& e0_joint_transform,
        const trs& e1_joint_transform,
        float twist_min_deg = -M_PI,
        float twist_max_deg = M_PI,
        float swing_y_deg = M_PI,
        float swing_z_deg = M_PI)
    {
        trs e0_joint_frame = e0_joint_transform;
        trs e1_joint_frame = e1_joint_transform;
        e0_joint_frame.position = e1_joint.joint_object_space_transform.position;
        e1_joint_frame.position = e1_joint.joint_object_space_transform.position;

        trs e0_to_e0_joint_transform = invert(e0_joint.joint_object_space_transform) * e0_joint_frame;
        e0_to_e0_joint_transform.rotation = normalize(e0_to_e0_joint_transform.rotation);
        e0_to_e0_joint_transform.scale = vec3(1.0f);

        trs e1_to_e1_joint_transform = invert(e1_joint.joint_object_space_transform) * e1_joint_frame;
        e1_to_e1_joint_transform.rotation = normalize(e1_to_e1_joint_transform.rotation);
        e1_to_e1_joint_transform.scale = vec3(1.0f);

        Entity e0 = e0_joint.physical_limb.value();
        Entity e1 = e1_joint.physical_limb.value();

        JointComponent::BaseDescriptor descriptor;
        descriptor.connected_entity = e0.get_data_weakref();
        descriptor.second_connected_entity = e1.get_data_weakref();
        descriptor.local_frame = e0_to_e0_joint_transform;
        descriptor.second_local_frame = e1_to_e1_joint_transform;

        Entity joint_entity = e0.get_world()->create_entity();
        joint_entity.add_component<TransientComponent>();
        joint_entity.set_parent(source.get_handle());

        D6JointComponent* joint_component = joint_entity.add_component<D6JointComponent>(descriptor);

        joint_component->enable_collision.get_for_write() = false;

        joint_component->linear_x_motion_type.get_for_write() = D6JointComponent::Motion::LOCKED;
        joint_component->linear_y_motion_type.get_for_write() = D6JointComponent::Motion::LOCKED;
        joint_component->linear_z_motion_type.get_for_write() = D6JointComponent::Motion::LOCKED;

        if (is_physiclly_animated)
        {
            e0.get_component<PhysicalAnimationLimbComponent>()->joint_entity_ptr = EntityPtr{joint_entity};
            e1.get_component<PhysicalAnimationLimbComponent>()->parent_joint_component = ComponentPtr{joint_component};
        }
        else
        {
            e0.get_component<RagdollLimbComponent>()->joint_entity_ptr = EntityPtr{joint_entity};
            e1.get_component<RagdollLimbComponent>()->parent_joint_component = ComponentPtr{joint_component};
        }

        if (fuzzy_equals(twist_max_deg, M_PI) && fuzzy_equals(twist_min_deg, -M_PI))
        {
            joint_component->twist_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
        }
        else if (fuzzy_equals(twist_max_deg - twist_min_deg, 0.0f))
        {
            joint_component->twist_motion_type.get_for_write() = D6JointComponent::Motion::LOCKED;
        }
        else
        {
            joint_component->twist_motion_type.get_for_write() = D6JointComponent::Motion::LIMITED;
            joint_component->twist_min_limit.get_for_write() = twist_min_deg;
            joint_component->twist_max_limit.get_for_write() = twist_max_deg;

            joint_component->twist_limit_restitution.get_for_write() = 0.0f;
        }

        bool any_moving_swing = false;

        if (fuzzy_equals(swing_y_deg, M_PI))
        {
            joint_component->swing_y_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
        }
        else if (fuzzy_equals(swing_y_deg, 0.0f))
        {
            joint_component->swing_y_motion_type.get_for_write() = D6JointComponent::Motion::LOCKED;
        }
        else
        {
            joint_component->swing_y_motion_type.get_for_write() = D6JointComponent::Motion::LIMITED;
            joint_component->swing_y_limit.get_for_write() = swing_y_deg;
            any_moving_swing = true;
        }

        if (fuzzy_equals(swing_z_deg, M_PI))
        {
            joint_component->swing_z_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
        }
        else if (fuzzy_equals(swing_z_deg, 0.0f))
        {
            joint_component->swing_z_motion_type.get_for_write() = D6JointComponent::Motion::LOCKED;
        }
        else
        {
            joint_component->swing_z_motion_type.get_for_write() = D6JointComponent::Motion::LIMITED;
            joint_component->swing_z_limit.get_for_write() = swing_z_deg;
            any_moving_swing = true;
        }

        // Swing limits stay as hard constraints. PhysX adds the cone limit row on every step, and a soft one becomes a spring that pulls the
        // swing towards the nearest point of the cone even inside it.
        if (any_moving_swing)
        {
            joint_component->swing_limit_damping.get_for_write() = 0.0f;
            joint_component->swing_limit_stiffness.get_for_write() = 0.0f;
            joint_component->swing_limit_restitution.get_for_write() = 0.0f;
        }
    }

    static quat make_bone_frame_rotation(const vec3& bone_direction, const vec3& up)
    {
        const vec3 x = normalize(bone_direction);
        const vec3 z = normalize(up - x * dot(up, x));
        const vec3 y = cross(z, x);
        return normalize(mat3_to_quaternion(mat3(
            x.x, y.x, z.x,
            x.y, y.y, z.y,
            x.z, y.z, z.z)));
    }

    static void create_drive_joint(
        const Entity& source,
        const PhysicalLimbDetails& details,
        Entity& e0,
        Entity& e1)
    {
        if (!details.motor_drive.has_value())
        {
            return;
        }

        JointComponent::BaseDescriptor descriptor;
        descriptor.connected_entity = e0.get_data_weakref();
        descriptor.second_connected_entity = e1.get_data_weakref();
        descriptor.local_frame = trs::identity;
        descriptor.second_local_frame = trs::identity;

        Entity joint_entity = e1.get_world()->create_entity();
        joint_entity.add_component<TransientComponent>();
        joint_entity.set_parent(source.get_handle());

        PhysicalAnimationLimbComponent* e1_limb_component = e1.get_component<PhysicalAnimationLimbComponent>();

        e1_limb_component->drive_joint_entity_ptr = EntityPtr{joint_entity};

        D6JointComponent* joint_component = joint_entity.add_component<D6JointComponent>(descriptor);

        joint_component->perform_slerp_drive = true;

        joint_component->enable_collision.get_for_write() = false;
        joint_component->drive_limits_are_forces.get_for_write() = true;

        // Improved slerp decouples the three slerp rows for non-spherical inertia (CPU solver).
        joint_component->improved_slerp.get_for_write() = true;
        joint_component->disable_preprocessing.get_for_write() = false;

        joint_component->linear_x_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
        joint_component->linear_y_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
        joint_component->linear_z_motion_type.get_for_write() = D6JointComponent::Motion::FREE;

        joint_component->twist_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
        joint_component->swing_y_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
        joint_component->swing_z_motion_type.get_for_write() = D6JointComponent::Motion::FREE;

        // Will be inited with profile values.
        joint_component->linear_drive_stiffness.get_for_write() = 0.0f;
        joint_component->linear_drive_damping.get_for_write() = 0.0f;
        joint_component->linear_drive_force_limit.get_for_write() = 0.0f;
        joint_component->linear_drive_accelerated.get_for_write() = true;

        joint_component->slerp_drive_stiffness.get_for_write() = 0.0f;
        joint_component->slerp_drive_damping.get_for_write() = 0.0f;
        joint_component->slerp_drive_force_limit.get_for_write() = 0.0f;
        joint_component->slerp_drive_accelerated.get_for_write() = true;

        e1_limb_component->drive_joint_component = ComponentPtr{joint_component};
    }

    // Return bottom of the box, used for creating d6s
    // Bottom = lowest point along length axis.
    static trs position_box_between_joints(
        BoxShapeComponent* box_shape_component,
        const vec3& size,
        const trs& from_joint_transform,
        const trs& to_joint_transform,
        const trs& owner_joint_transform,
        const vec3& local_position_adjustment = vec3::zero,
        const quat& local_rotation_spin = quat::identity)
    {
        box_shape_component->half_extents = (size / 2.0f) / owner_joint_transform.scale.x;

        ASSERT(squared_length(box_shape_component->half_extents) > 0.0f);

        const vec3 offset = to_joint_transform.position - from_joint_transform.position;
        const vec3 center_between_joints = from_joint_transform.position + offset / 2.0f;
        const vec3 direction = normalize(offset);

        const trs box_transform(
            center_between_joints + local_position_adjustment,
            shortest_arc(vec3::right, direction) * local_rotation_spin,
            vec3(1.0f));
        const trs box_transform_relative_to_owner = invert(owner_joint_transform) * box_transform;

        box_shape_component->local_position.get_for_write() = box_transform_relative_to_owner.position;
        box_shape_component->local_rotation.get_for_write() = box_transform_relative_to_owner.rotation;

        return trs(box_transform.position - direction * size.x / 2.0f, box_transform.rotation, box_transform.scale);
    }

    // Return bottom of the capsule, used for creating d6s
    // Bottom = lowest point on x axis. The capsule starts at the first joint, height_multiplier only moves its far end.
    static trs position_capsule_between_joints_from_radius(
        CapsuleShapeComponent* capsule_shape_component,
        const float radius,
        const trs& from_joint_transform,
        const trs& to_joint_transform,
        const trs& owner_joint_transform,
        const float height_multiplier,
        const vec3& local_position_adjustment = vec3::zero,
        const quat& local_rotation_spin = quat::identity)
    {
        const vec3 offset = to_joint_transform.position - from_joint_transform.position;
        const float height = (length(offset) - 2 * radius) * height_multiplier;

        capsule_shape_component->radius = radius / owner_joint_transform.scale.x;
        capsule_shape_component->half_height = (height / 2.0f) / owner_joint_transform.scale.x;

        ASSERT(capsule_shape_component->radius > 0.0f);
        ASSERT(capsule_shape_component->half_height > 0.0f);

        const vec3 direction = normalize(offset);
        const vec3 capsule_center = from_joint_transform.position + direction * (radius + height / 2.0f);

        const trs capsule_transform(
            capsule_center + local_position_adjustment,
            shortest_arc(vec3::right, direction) * local_rotation_spin,
            vec3(1.0f));
        const trs capsule_transform_relative_to_owner = invert(owner_joint_transform) * capsule_transform;

        capsule_shape_component->local_position.get_for_write() = capsule_transform_relative_to_owner.position;
        capsule_shape_component->local_rotation.get_for_write() = capsule_transform_relative_to_owner.rotation;

        return trs(capsule_transform.position - direction * (radius + height / 2.0f), capsule_transform.rotation, capsule_transform.scale);
    }

    // Return bottom of the capsule, used for creating d6s
    // Bottom = lowest point on x axis.
    static trs position_capsule(
        CapsuleShapeComponent* capsule_shape_component,
        const float radius,
        const float half_height,
        const trs& from_joint_transform,
        const trs& to_joint_transform,
        const trs& owner_joint_transform,
        const vec3& local_position_adjustment = vec3::zero,
        const quat& local_rotation_spin = quat::identity)
    {
        const vec3 offset = to_joint_transform.position - from_joint_transform.position;

        capsule_shape_component->radius = radius / owner_joint_transform.scale.x;
        capsule_shape_component->half_height = half_height / owner_joint_transform.scale.x;

        ASSERT(capsule_shape_component->radius > 0.0f);
        ASSERT(capsule_shape_component->half_height > 0.0f);

        const vec3 center_between_joints = from_joint_transform.position + offset / 2.0f;
        const vec3 direction = normalize(offset);

        const trs capsule_transform(
            center_between_joints + local_position_adjustment,
            shortest_arc(vec3::right, direction) * local_rotation_spin,
            vec3(1.0f));
        const trs capsule_transform_relative_to_owner = invert(owner_joint_transform) * capsule_transform;

        capsule_shape_component->local_position.get_for_write() = capsule_transform_relative_to_owner.position;
        capsule_shape_component->local_rotation.get_for_write() = capsule_transform_relative_to_owner.rotation;

        return trs(capsule_transform.position - direction * (radius + half_height), capsule_transform.rotation, capsule_transform.scale);
    }

    // Return bottom of the capsule, used for creating d6s
    // Bottom = x is along the diameter, lowest point on x axis.
    static trs position_capsule_between_joints_from_height(
        CapsuleShapeComponent* capsule_shape_component,
        float height,
        const trs& from_joint_transform,
        const trs& to_joint_transform,
        const trs& owner_joint_transform,
        const float radius_modifier = 1.0f,
        const vec3& local_position_adjustment = vec3::zero,
        const quat& local_rotation_spin = quat::identity)
    {
        const vec3 offset = to_joint_transform.position - from_joint_transform.position;
        const float radius = length(offset) * 0.65f * radius_modifier; // Increased intentionally for thorax, abdomen and pelvis. Should probably be specified outside.
        if (height - 2.0f * radius > EPSILON)
        {
            height -= 2.0f * radius;
        }

        capsule_shape_component->radius = radius / owner_joint_transform.scale.x;
        capsule_shape_component->half_height = (height / 2.0f) / owner_joint_transform.scale.x;

        ASSERT(capsule_shape_component->radius > 0.0f);
        ASSERT(capsule_shape_component->half_height > 0.0f);

        const vec3 center_between_joints = from_joint_transform.position + offset / 2.0f;
        const vec3 direction = normalize(offset);

        const trs capsule_transform(
            center_between_joints + local_position_adjustment,
            shortest_arc(vec3::up, direction) * local_rotation_spin,
            vec3(1.0f));
        const trs capsule_transform_relative_to_owner = invert(owner_joint_transform) * capsule_transform;

        capsule_shape_component->local_position.get_for_write() = capsule_transform_relative_to_owner.position;
        capsule_shape_component->local_rotation.get_for_write() = capsule_transform_relative_to_owner.rotation;

        const vec3 capsule_x_axis = capsule_transform.rotation * vec3::right;
        return trs(capsule_transform.position - direction * radius, capsule_transform.rotation, capsule_transform.scale) * trs(vec3::zero, shortest_arc(capsule_x_axis, direction), vec3(1.0f));
    }

    void PhysicalRagdollBuilder::build_ragdoll(const RuntimeContext& ctx)
    {
        if (ctx.ragdoll_component->mass <= 0.0f)
        {
            ASSERT(false);
            return;
        }

        const float total_mass_percentage = ctx.ragdoll_component->settings.mass_settings.get_total_percentage();
        if (std::abs(total_mass_percentage - 1.0f) > 0.01f)
        {
            LOG_WARNING("Ragdoll builder> Mass percentages sum to %.3f instead of 1, the ragdoll weighs %.1f kg instead of %.1f kg.",
                total_mass_percentage, ctx.ragdoll_component->mass * total_mass_percentage, ctx.ragdoll_component->mass);
        }

        if (ctx.enable_physical_animation)
        {
            ctx.ragdoll.add_component<PhysicalAnimationComponent>();
        }
        else
        {
            ctx.ragdoll.add_component<RagdollComponent>();
        }

        bool status = prepare_structure(ctx);
        if (!status)
        {
            ASSERT(status);
            return;
        }

        status |= build_bodies(ctx);
        if (!status)
        {
            ASSERT(status);
            return;
        }

        status |= build_constraint_joints(ctx);
        if (!status)
        {
            ASSERT(status);
            return;
        }

        status |= build_kinematics(ctx);
        if (!status)
        {
            ASSERT(status);
            return;
        }

        status |= build_drive_joints(ctx);
        if (!status)
        {
            ASSERT(status);
        }
    }

    bool PhysicalRagdollBuilder::prepare_structure(const RuntimeContext& ctx)
    {
        using namespace animation;

        const SkeletonComponent* skeleton_component = ctx.ragdoll.get_component<SkeletonComponent>();
        if (skeleton_component == nullptr ||
            skeleton_component->skeleton == nullptr ||
            skeleton_component->skeleton->load_state != AssetLoadState::LOADED)
        {
            ASSERT(false);
            return false;
        }

        const Skeleton* skeleton = skeleton_component->skeleton.get();
        const SkeletonPose& reference_pose = skeleton->get_default_pose();

        auto prepare_joint = [&reference_pose, skeleton](RagdollSkeletonStructure::JointStructure& joint, uint32 joint_id)
        {
            joint.joint_id = joint_id;
            joint.joint_object_space_transform = SkeletonUtils::get_object_space_joint_transform(reference_pose, skeleton, joint.joint_id);
        };

        prepare_joint(ctx.skeleton_structure.head_end_joint, ctx.ragdoll_component->joint_init_ids.head_end_idx);
        prepare_joint(ctx.skeleton_structure.head_joint, ctx.ragdoll_component->joint_init_ids.head_idx);
        prepare_joint(ctx.skeleton_structure.neck_joint, ctx.ragdoll_component->joint_init_ids.neck_idx);

        prepare_joint(ctx.skeleton_structure.thorax_joint, ctx.ragdoll_component->joint_init_ids.spine_03_idx);
        prepare_joint(ctx.skeleton_structure.abdomen_joint, ctx.ragdoll_component->joint_init_ids.spine_01_idx);
        prepare_joint(ctx.skeleton_structure.pelvis_joint, ctx.ragdoll_component->joint_init_ids.pelvis_idx);

        prepare_joint(ctx.skeleton_structure.left_clavicle_joint, ctx.ragdoll_component->joint_init_ids.clavicle_l_idx);
        prepare_joint(ctx.skeleton_structure.left_arm_joint, ctx.ragdoll_component->joint_init_ids.upperarm_l_idx);
        prepare_joint(ctx.skeleton_structure.left_forearm_joint, ctx.ragdoll_component->joint_init_ids.lowerarm_l_idx);
        prepare_joint(ctx.skeleton_structure.left_hand_joint, ctx.ragdoll_component->joint_init_ids.hand_l_idx);
        prepare_joint(ctx.skeleton_structure.left_hand_end_joint, ctx.ragdoll_component->joint_init_ids.hand_end_l_idx);

        prepare_joint(ctx.skeleton_structure.right_clavicle_joint, ctx.ragdoll_component->joint_init_ids.clavicle_r_idx);
        prepare_joint(ctx.skeleton_structure.right_arm_joint, ctx.ragdoll_component->joint_init_ids.upperarm_r_idx);
        prepare_joint(ctx.skeleton_structure.right_forearm_joint, ctx.ragdoll_component->joint_init_ids.lowerarm_r_idx);
        prepare_joint(ctx.skeleton_structure.right_hand_joint, ctx.ragdoll_component->joint_init_ids.hand_r_idx);
        prepare_joint(ctx.skeleton_structure.right_hand_end_joint, ctx.ragdoll_component->joint_init_ids.hand_end_r_idx);

        prepare_joint(ctx.skeleton_structure.left_leg_joint, ctx.ragdoll_component->joint_init_ids.thigh_l_idx);
        prepare_joint(ctx.skeleton_structure.left_calf_joint, ctx.ragdoll_component->joint_init_ids.calf_l_idx);
        prepare_joint(ctx.skeleton_structure.left_foot_joint, ctx.ragdoll_component->joint_init_ids.foot_l_idx);
        prepare_joint(ctx.skeleton_structure.left_foot_end_joint, ctx.ragdoll_component->joint_init_ids.foot_end_l_idx);

        prepare_joint(ctx.skeleton_structure.right_leg_joint, ctx.ragdoll_component->joint_init_ids.thigh_r_idx);
        prepare_joint(ctx.skeleton_structure.right_calf_joint, ctx.ragdoll_component->joint_init_ids.calf_r_idx);
        prepare_joint(ctx.skeleton_structure.right_foot_joint, ctx.ragdoll_component->joint_init_ids.foot_r_idx);
        prepare_joint(ctx.skeleton_structure.right_foot_end_joint, ctx.ragdoll_component->joint_init_ids.foot_end_r_idx);

        ctx.skeleton_structure.distance_between_arms = distance(ctx.skeleton_structure.left_arm_joint.joint_object_space_transform.position,
            ctx.skeleton_structure.right_arm_joint.joint_object_space_transform.position);

        ctx.skeleton_structure.distance_between_hand_and_hand_end = distance(ctx.skeleton_structure.left_hand_joint.joint_object_space_transform.position,
            ctx.skeleton_structure.left_hand_end_joint.joint_object_space_transform.position);

        ctx.skeleton_structure.distance_between_foot_and_foot_end = distance(ctx.skeleton_structure.left_foot_joint.joint_object_space_transform.position,
                                                                        ctx.skeleton_structure.left_foot_end_joint.joint_object_space_transform.position) +
                                                                    0.04f;
        ctx.skeleton_structure.distance_between_foot_y_and_foot_end_y = abs(ctx.skeleton_structure.left_foot_joint.joint_object_space_transform.position.y -
                                                                            ctx.skeleton_structure.left_foot_end_joint.joint_object_space_transform.position.y) -
                                                                        0.03f;

        return true;
    }

    bool PhysicalRagdollBuilder::build_bodies(const RuntimeContext& ctx)
    {
        using namespace animation;

        if (!PhysicsEngine::get_physics_core()->is_gpu())
        {
            // GPU broad phase does not support PxAggregate.
            AggregateHolderComponent* aggregate_component = ctx.ragdoll.add_component<AggregateHolderComponent>();
            aggregate_component->enable_self_collision = true;
            aggregate_component->max_actors = 48;
        }

        const RagdollSettings& settings = ctx.ragdoll_component->settings;

        auto create_limb_material = [&settings](float static_friction, float dynamic_friction)
        {
            ref<PhysicsMaterial> limb_material = PhysicsEngine::get_physics_core()->create_material(
                settings.material_settings.restitution, static_friction, dynamic_friction);
            ASSERT(limb_material != nullptr);
            limb_material->set_restitution_combine_mode(PhysicsMaterialCombineMode::MIN);
            return limb_material;
        };

        const ref<PhysicsMaterial> body_material = create_limb_material(settings.material_settings.static_friction, settings.material_settings.dynamic_friction);
        const ref<PhysicsMaterial> foot_material = create_limb_material(settings.material_settings.foot_static_friction, settings.material_settings.foot_dynamic_friction);
        const ref<PhysicsMaterial> hand_material = create_limb_material(settings.material_settings.hand_static_friction, settings.material_settings.hand_dynamic_friction);

        auto get_limb_material = [&body_material, &foot_material, &hand_material](RagdollLimbType limb_type) -> const ref<PhysicsMaterial>&
        {
            if (limb_type == RagdollLimbType::FOOT)
            {
                return foot_material;
            }
            if (limb_type == RagdollLimbType::HAND)
            {
                return hand_material;
            }
            return body_material;
        };
        RagdollSkeletonStructure& structure = ctx.skeleton_structure;

        const SkeletonComponent* skeleton_component = ctx.ragdoll.get_component<SkeletonComponent>();

        auto create_capsule_limb_between_joints = [&ctx, &get_limb_material, &skeleton_component](
                                                      uint32 joint_id,
                                                      const PhysicalLimbDetails* details,
                                                      float mass_percentage,
                                                      RagdollLimbType limb_type,
                                                      RagdollSkeletonStructure::JointStructure& joint_structure,
                                                      const RagdollSkeletonStructure::JointStructure& connected_joint_structure,
                                                      float measure,
                                                      float modifier,
                                                      bool create_from_radius = true,
                                                      PhysicsLimbChain* chain = nullptr,
                                                      CollisionType collision_type = CollisionType::RAGDOLL,
                                                      const vec3& first_joint_adjustment = vec3::zero,
                                                      const vec3& second_joint_adjustment = vec3::zero,
                                                      const vec3& constraint_joint_adjustment = vec3::zero,
                                                      const quat& constraint_joint_spin = quat::identity)
        {
            Entity limb = create_child_entity(ctx.enable_physical_animation,
                ctx.ragdoll,
                ctx.ragdoll_component,
                skeleton_component->skeleton->joints[joint_id].name,
                details,
                joint_id,
                limb_type,
                chain);

            joint_structure.physical_limb = limb;

            trs joint_transform = joint_structure.joint_object_space_transform;
            joint_transform.position += first_joint_adjustment;

            trs low_joint_transform = connected_joint_structure.joint_object_space_transform;
            low_joint_transform.position += second_joint_adjustment;

            TransformComponent* transform_component = limb.get_component<TransformComponent>();
            transform_component->set_local_transform(joint_transform);

            CapsuleShapeComponent* capsule_shape_component = create_shape<CapsuleShapeComponent>(limb, get_limb_material(limb_type), collision_type);

            if (create_from_radius)
            {
                joint_structure.constraint_object_space_transform = position_capsule_between_joints_from_radius(capsule_shape_component,
                    measure,
                    joint_transform,
                    low_joint_transform,
                    joint_transform,
                    modifier,
                    constraint_joint_adjustment,
                    constraint_joint_spin);
            }
            else
            {

                joint_structure.constraint_object_space_transform = position_capsule_between_joints_from_height(capsule_shape_component,
                    measure,
                    joint_transform,
                    low_joint_transform,
                    joint_transform,
                    modifier,
                    constraint_joint_adjustment,
                    constraint_joint_spin);
            }

            create_dynamic_body(
                ctx.enable_physical_animation,
                limb,
                ctx.ragdoll_component->mass * mass_percentage,
                ctx.ragdoll_component->settings);
        };

        auto create_box_limb_between_joints = [&ctx, &get_limb_material, &skeleton_component](
                                                  uint32 joint_id,
                                                  const PhysicalLimbDetails* details,
                                                  float mass_percentage,
                                                  RagdollLimbType limb_type,
                                                  RagdollSkeletonStructure::JointStructure& joint_structure,
                                                  const RagdollSkeletonStructure::JointStructure& connected_joint_structure,
                                                  const vec3& size,
                                                  PhysicsLimbChain* chain = nullptr,
                                                  CollisionType collision_type = CollisionType::RAGDOLL,
                                                  const vec3& first_joint_adjustment = vec3::zero,
                                                  const vec3& second_joint_adjustment = vec3::zero,
                                                  const vec3& constraint_joint_adjustment = vec3::zero,
                                                  const quat& constraint_joint_spin = quat::identity)
        {
            Entity limb = create_child_entity(ctx.enable_physical_animation,
                ctx.ragdoll,
                ctx.ragdoll_component,
                skeleton_component->skeleton->joints[joint_id].name,
                details,
                joint_id,
                limb_type,
                chain);

            joint_structure.physical_limb = limb;

            trs joint_transform = joint_structure.joint_object_space_transform;
            joint_transform.position += first_joint_adjustment;

            trs low_joint_transform = connected_joint_structure.joint_object_space_transform;
            low_joint_transform.position += second_joint_adjustment;

            TransformComponent* transform_component = limb.get_component<TransformComponent>();
            transform_component->set_local_transform(joint_transform);

            BoxShapeComponent* box_shape_component = create_shape<BoxShapeComponent>(limb, get_limb_material(limb_type), collision_type);

            joint_structure.constraint_object_space_transform = position_box_between_joints(box_shape_component,
                size,
                joint_transform,
                low_joint_transform,
                joint_transform,
                constraint_joint_adjustment,
                constraint_joint_spin);

            create_dynamic_body(
                ctx.enable_physical_animation,
                limb,
                ctx.ragdoll_component->mass * mass_percentage,
                ctx.ragdoll_component->settings);
        };

        auto create_capsule_limb_between_joints_from_radius = [&ctx, &get_limb_material, &skeleton_component](
                                                                  uint32 joint_id,
                                                                  const PhysicalLimbDetails* details,
                                                                  float mass_percentage,
                                                                  RagdollLimbType limb_type,
                                                                  RagdollSkeletonStructure::JointStructure& joint_structure,
                                                                  const RagdollSkeletonStructure::JointStructure& up_joint_structure,
                                                                  const RagdollSkeletonStructure::JointStructure& low_joint_structure,
                                                                  float height,
                                                                  float radius_modifier,
                                                                  float middle_interpolaton_factor,
                                                                  bool is_position_between,
                                                                  PhysicsLimbChain* chain = nullptr,
                                                                  CollisionType collision_type = CollisionType::RAGDOLL,
                                                                  const vec3& up_joint_adjustment = vec3::zero,
                                                                  const vec3& low_joint_adjustment = vec3::zero,
                                                                  const vec3& constraint_joint_adjustment = vec3::zero,
                                                                  const quat& constraint_joint_spin = quat::identity)
        {
            Entity limb = create_child_entity(ctx.enable_physical_animation,
                ctx.ragdoll,
                ctx.ragdoll_component,
                skeleton_component->skeleton->joints[joint_id].name,
                details,
                joint_id,
                limb_type,
                chain);

            joint_structure.physical_limb = limb;

            trs joint_transform = joint_structure.joint_object_space_transform;

            trs up_joint_transform = up_joint_structure.joint_object_space_transform;
            up_joint_transform.position += up_joint_adjustment;

            trs low_joint_transform = low_joint_structure.joint_object_space_transform;
            low_joint_transform.position += low_joint_adjustment;

            TransformComponent* transform_component = limb.get_component<TransformComponent>();
            transform_component->set_local_transform(joint_transform);

            CapsuleShapeComponent* capsule_shape_component = create_shape<CapsuleShapeComponent>(limb, get_limb_material(limb_type), collision_type);

            const trs middle_transform = trs(
                lerp(low_joint_transform.position, joint_transform.position, middle_interpolaton_factor),
                slerp(low_joint_transform.rotation, joint_transform.rotation, middle_interpolaton_factor),
                lerp(low_joint_transform.scale, joint_transform.scale, middle_interpolaton_factor));

            if (is_position_between)
            {
                joint_structure.constraint_object_space_transform = position_capsule_between_joints_from_height(capsule_shape_component,
                    height,
                    middle_transform,
                    up_joint_transform,
                    joint_transform,
                    radius_modifier,
                    constraint_joint_adjustment,
                    constraint_joint_spin);
            }
            else
            {
                joint_structure.constraint_object_space_transform = position_capsule_between_joints_from_height(capsule_shape_component,
                    height,
                    joint_transform,
                    middle_transform,
                    joint_transform,
                    radius_modifier,
                    constraint_joint_adjustment,
                    constraint_joint_spin);
            }

            create_dynamic_body(
                ctx.enable_physical_animation,
                limb,
                ctx.ragdoll_component->mass * mass_percentage,
                ctx.ragdoll_component->settings);
        };

        PhysicalAnimationComponent* physical_animation_component = ctx.enable_physical_animation ? dynamic_cast<PhysicalAnimationComponent*>(ctx.ragdoll_component) : nullptr;

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.neck_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->neck_limb_details : nullptr,
            settings.mass_settings.neck_mass_percentage,
            RagdollLimbType::NECK,
            structure.neck_joint,
            structure.head_joint,
            settings.shapes_settings.neck_radius,
            settings.scaler_settings.neck_height_modifier,
            true,
            ctx.enable_physical_animation ? physical_animation_component->neck_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.neck_joint_adjastment,
            settings.object_space_settings.head_joint_adjastment,
            settings.local_shape_settings.neck_joint_adjastment,
            settings.local_shape_settings.neck_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.head_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->head_limb_details : nullptr,
            settings.mass_settings.head_mass_percentage,
            RagdollLimbType::HEAD,
            structure.head_joint,
            structure.head_end_joint,
            settings.shapes_settings.head_radius,
            1.0f,
            true,
            ctx.enable_physical_animation ? physical_animation_component->neck_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.head_joint_adjastment,
            settings.object_space_settings.head_end_joint_adjastment,
            settings.local_shape_settings.head_joint_adjastment,
            settings.local_shape_settings.head_joint_spin);

        create_capsule_limb_between_joints_from_radius(
            ctx.ragdoll_component->joint_init_ids.spine_03_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->body_upper_limb_details : nullptr,
            settings.mass_settings.body_upper_mass_percentage,
            RagdollLimbType::BODY_UPPER,
            structure.thorax_joint,
            structure.neck_joint,
            structure.abdomen_joint,
            structure.distance_between_arms * settings.scaler_settings.upper_body_height_modifier,
            settings.scaler_settings.upper_body_radius_modifier,
            0.7f,
            true,
            ctx.enable_physical_animation ? physical_animation_component->body_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.neck_joint_adjastment,
            settings.object_space_settings.abdomen_joint_adjastment,
            settings.local_shape_settings.thorax_joint_adjastment,
            settings.local_shape_settings.thorax_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.spine_01_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->body_middle_limb_details : nullptr,
            settings.mass_settings.body_middle_mass_percentage,
            RagdollLimbType::BODY_MIDDLE,
            structure.abdomen_joint,
            structure.thorax_joint,
            structure.distance_between_arms * settings.scaler_settings.middle_body_height_modifier,
            settings.scaler_settings.middle_body_radius_modifier,
            false,
            ctx.enable_physical_animation ? physical_animation_component->body_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.abdomen_joint_adjastment,
            settings.object_space_settings.thorax_joint_adjastment,
            settings.local_shape_settings.abdomen_joint_adjastment,
            settings.local_shape_settings.abdomen_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.pelvis_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->body_lower_limb_details : nullptr,
            settings.mass_settings.body_lower_mass_percentage,
            RagdollLimbType::BODY_LOWER,
            structure.pelvis_joint,
            structure.abdomen_joint,
            structure.distance_between_arms * settings.scaler_settings.lower_body_height_modifier,
            settings.scaler_settings.lower_body_radius_modifier,
            false,
            ctx.enable_physical_animation ? physical_animation_component->body_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.pelvis_joint_adjastment,
            settings.object_space_settings.abdomen_joint_adjastment,
            settings.local_shape_settings.pelvis_joint_adjastment,
            settings.local_shape_settings.pelvis_joint_spin);

        // Lower body attachment.
        if (ctx.enable_physical_animation)
        {
            const trs& ragdoll_world_transform = ctx.ragdoll.get_component<TransformComponent>()->get_world_transform();

            Entity body_lower = structure.pelvis_joint.physical_limb.value();

            Entity body_lower_ghost = create_attachment_dynamic_body(ctx.ragdoll,
                "PelvisAttachment",
                body_lower.get_component<DynamicBodyComponent>()->mass);

            TransformComponent* transform_component = body_lower_ghost.get_component<TransformComponent>();
            transform_component->set_world_transform(ragdoll_world_transform * structure.pelvis_joint.joint_object_space_transform);

            JointComponent::BaseDescriptor descriptor;
            descriptor.connected_entity = body_lower_ghost.get_data_weakref();
            descriptor.second_connected_entity = body_lower.get_data_weakref();

            if (physical_animation_component->use_fixed_pelvis_attachment)
            {
                FixedJointComponent* joint_component = body_lower_ghost.add_component<FixedJointComponent>(descriptor);
                joint_component->enable_collision.get_for_write() = false;
            }
            else
            {
                DistanceJointComponent* joint_component = body_lower_ghost.add_component<DistanceJointComponent>(descriptor);
                joint_component->enable_collision.get_for_write() = false;
                joint_component->spring_enabled.get_for_write() = true;
            }

            physical_animation_component->attachment_body = EntityPtr{body_lower_ghost};
        }

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.clavicle_l_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->clavicle_limb_details : nullptr,
            settings.mass_settings.clavicle_mass_percentage,
            RagdollLimbType::CLAVICLE,
            structure.left_clavicle_joint,
            structure.left_arm_joint,
            settings.shapes_settings.clavicle_radius,
            settings.scaler_settings.clavicle_height_modifier,
            true,
            ctx.enable_physical_animation ? physical_animation_component->left_arm_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.left_clavicle_joint_adjastment,
            settings.object_space_settings.left_arm_joint_adjastment,
            settings.local_shape_settings.left_clavicle_joint_adjastment,
            settings.local_shape_settings.left_clavicle_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.upperarm_l_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->arm_limb_details : nullptr,
            settings.mass_settings.arm_mass_percentage,
            RagdollLimbType::ARM,
            structure.left_arm_joint,
            structure.left_forearm_joint,
            settings.shapes_settings.arm_radius,
            settings.scaler_settings.arm_height_modifier,
            true,
            ctx.enable_physical_animation ? physical_animation_component->left_arm_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.left_arm_joint_adjastment,
            settings.object_space_settings.left_forearm_joint_adjastment,
            settings.local_shape_settings.left_arm_joint_adjastment,
            settings.local_shape_settings.left_arm_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.lowerarm_l_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->forearm_limb_details : nullptr,
            settings.mass_settings.forearm_mass_percentage,
            RagdollLimbType::FOREARM,
            structure.left_forearm_joint,
            structure.left_hand_joint,
            settings.shapes_settings.forearm_radius,
            settings.scaler_settings.forearm_height_modifier,
            true,
            ctx.enable_physical_animation ? physical_animation_component->left_arm_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.left_forearm_joint_adjastment,
            settings.object_space_settings.left_hand_joint_adjastment,
            settings.local_shape_settings.left_hand_joint_adjastment,
            settings.local_shape_settings.left_hand_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.hand_l_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->hand_limb_details : nullptr,
            settings.mass_settings.hand_mass_percentage,
            RagdollLimbType::HAND,
            structure.left_hand_joint,
            structure.left_hand_end_joint,
            settings.shapes_settings.hand_radius,
            0.6f,
            true,
            ctx.enable_physical_animation ? physical_animation_component->left_arm_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.left_hand_joint_adjastment,
            vec3::zero,
            settings.local_shape_settings.left_hand_joint_adjastment,
            settings.local_shape_settings.left_hand_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.clavicle_r_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->clavicle_limb_details : nullptr,
            settings.mass_settings.clavicle_mass_percentage,
            RagdollLimbType::CLAVICLE,
            structure.right_clavicle_joint,
            structure.right_arm_joint,
            settings.shapes_settings.clavicle_radius,
            settings.scaler_settings.clavicle_height_modifier,
            true,
            ctx.enable_physical_animation ? physical_animation_component->right_arm_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.right_clavicle_joint_adjastment,
            settings.object_space_settings.right_arm_joint_adjastment,
            settings.local_shape_settings.right_clavicle_joint_adjastment,
            settings.local_shape_settings.right_clavicle_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.upperarm_r_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->arm_limb_details : nullptr,
            settings.mass_settings.arm_mass_percentage,
            RagdollLimbType::ARM,
            structure.right_arm_joint,
            structure.right_forearm_joint,
            settings.shapes_settings.arm_radius,
            settings.scaler_settings.arm_height_modifier,
            true,
            ctx.enable_physical_animation ? physical_animation_component->right_arm_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.right_arm_joint_adjastment,
            settings.object_space_settings.right_forearm_joint_adjastment,
            settings.local_shape_settings.right_arm_joint_adjastment,
            settings.local_shape_settings.right_arm_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.lowerarm_r_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->forearm_limb_details : nullptr,
            settings.mass_settings.forearm_mass_percentage,
            RagdollLimbType::FOREARM,
            structure.right_forearm_joint,
            structure.right_hand_joint,
            settings.shapes_settings.forearm_radius,
            settings.scaler_settings.forearm_height_modifier,
            true,
            ctx.enable_physical_animation ? physical_animation_component->right_arm_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.right_forearm_joint_adjastment,
            settings.object_space_settings.right_hand_joint_adjastment,
            settings.local_shape_settings.right_hand_joint_adjastment,
            settings.local_shape_settings.right_hand_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.hand_r_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->hand_limb_details : nullptr,
            settings.mass_settings.hand_mass_percentage,
            RagdollLimbType::HAND,
            structure.right_hand_joint,
            structure.right_hand_end_joint,
            settings.shapes_settings.hand_radius,
            0.6f,
            true,
            ctx.enable_physical_animation ? physical_animation_component->right_arm_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.right_hand_joint_adjastment,
            vec3::zero,
            settings.local_shape_settings.right_hand_joint_adjastment,
            settings.local_shape_settings.right_hand_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.thigh_l_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->leg_limb_details : nullptr,
            settings.mass_settings.leg_mass_percentage,
            RagdollLimbType::LEG,
            structure.left_leg_joint,
            structure.left_calf_joint,
            settings.shapes_settings.leg_radius,
            1.0f,
            true,
            ctx.enable_physical_animation ? physical_animation_component->left_leg_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.left_thigh_joint_adjastment,
            settings.object_space_settings.left_calf_joint_adjastment,
            settings.local_shape_settings.left_thigh_joint_adjastment,
            settings.local_shape_settings.left_thigh_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.calf_l_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->calf_limb_details : nullptr,
            settings.mass_settings.calf_mass_percentage,
            RagdollLimbType::CALF,
            structure.left_calf_joint,
            structure.left_foot_joint,
            settings.shapes_settings.calf_radius,
            1.0f,
            true,
            ctx.enable_physical_animation ? physical_animation_component->left_leg_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.left_calf_joint_adjastment,
            settings.object_space_settings.left_foot_joint_adjastment,
            settings.local_shape_settings.left_calf_joint_adjastment,
            settings.local_shape_settings.left_calf_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.foot_l_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->foot_limb_details : nullptr,
            settings.mass_settings.foot_mass_percentage,
            RagdollLimbType::FOOT,
            structure.left_foot_joint,
            structure.left_foot_end_joint,
            settings.shapes_settings.foot_radius,
            0.8f,
            true,
            ctx.enable_physical_animation ? physical_animation_component->left_leg_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.left_foot_joint_adjastment,
            settings.object_space_settings.left_foot_end_joint_adjastment,
            settings.local_shape_settings.left_foot_joint_adjastment,
            settings.local_shape_settings.left_foot_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.thigh_r_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->leg_limb_details : nullptr,
            settings.mass_settings.leg_mass_percentage,
            RagdollLimbType::LEG,
            structure.right_leg_joint,
            structure.right_calf_joint,
            settings.shapes_settings.leg_radius,
            1.0f,
            true,
            ctx.enable_physical_animation ? physical_animation_component->right_leg_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.right_thigh_joint_adjastment,
            settings.object_space_settings.right_calf_joint_adjastment,
            settings.local_shape_settings.right_thigh_joint_adjastment,
            settings.local_shape_settings.right_thigh_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.calf_r_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->calf_limb_details : nullptr,
            settings.mass_settings.calf_mass_percentage,
            RagdollLimbType::CALF,
            structure.right_calf_joint,
            structure.right_foot_joint,
            settings.shapes_settings.calf_radius,
            1.0f,
            true,
            ctx.enable_physical_animation ? physical_animation_component->right_leg_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.right_calf_joint_adjastment,
            settings.object_space_settings.right_foot_joint_adjastment,
            settings.local_shape_settings.right_calf_joint_adjastment,
            settings.local_shape_settings.right_calf_joint_spin);

        create_capsule_limb_between_joints(
            ctx.ragdoll_component->joint_init_ids.foot_r_idx,
            ctx.enable_physical_animation ? &ctx.default_profile->foot_limb_details : nullptr,
            settings.mass_settings.foot_mass_percentage,
            RagdollLimbType::FOOT,
            structure.right_foot_joint,
            structure.right_foot_end_joint,
            settings.shapes_settings.foot_radius,
            0.8f,
            true,
            ctx.enable_physical_animation ? physical_animation_component->right_leg_chain.get() : nullptr,
            CollisionType::RAGDOLL,
            settings.object_space_settings.right_foot_joint_adjastment,
            settings.object_space_settings.right_foot_end_joint_adjastment,
            settings.local_shape_settings.right_foot_joint_adjastment,
            settings.local_shape_settings.right_foot_joint_spin);

        return true;
    }

    bool PhysicalRagdollBuilder::build_kinematics(const RuntimeContext& ctx)
    {
        if (!ctx.enable_physical_animation)
        {
            return true;
        }

        if (ctx.default_profile == nullptr)
        {
            return false;
        }

        auto optional_build_kinematic_target = [&ctx](const PhysicalLimbDetails& limb_details,
                                                   const float mass_percentage,
                                                   RagdollSkeletonStructure::JointStructure& joint)
        {
            if (!limb_details.motor_drive.has_value())
            {
                return;
            }

            joint.kinematic_target = create_attachment_dynamic_body(ctx.ragdoll, "KinematicTarget", ctx.ragdoll_component->mass * mass_percentage);
            joint.kinematic_target->get_component<TransformComponent>()->set_local_transform(joint.joint_object_space_transform);
        };

        const RagdollProfile* profile = ctx.default_profile;
        const RagdollSettings::MassSettings& mass_settings = ctx.ragdoll_component->settings.mass_settings;

        optional_build_kinematic_target(profile->head_limb_details, mass_settings.head_mass_percentage, ctx.skeleton_structure.head_joint);
        optional_build_kinematic_target(profile->neck_limb_details, mass_settings.neck_mass_percentage, ctx.skeleton_structure.neck_joint);

        optional_build_kinematic_target(profile->body_upper_limb_details, mass_settings.body_upper_mass_percentage, ctx.skeleton_structure.thorax_joint);
        optional_build_kinematic_target(profile->body_middle_limb_details, mass_settings.body_middle_mass_percentage, ctx.skeleton_structure.abdomen_joint);
        optional_build_kinematic_target(profile->body_lower_limb_details, mass_settings.body_lower_mass_percentage, ctx.skeleton_structure.pelvis_joint);

        optional_build_kinematic_target(profile->clavicle_limb_details, mass_settings.clavicle_mass_percentage, ctx.skeleton_structure.left_clavicle_joint);
        optional_build_kinematic_target(profile->arm_limb_details, mass_settings.arm_mass_percentage, ctx.skeleton_structure.left_arm_joint);
        optional_build_kinematic_target(profile->forearm_limb_details, mass_settings.forearm_mass_percentage, ctx.skeleton_structure.left_forearm_joint);
        optional_build_kinematic_target(profile->hand_limb_details, mass_settings.hand_mass_percentage, ctx.skeleton_structure.left_hand_joint);

        optional_build_kinematic_target(profile->clavicle_limb_details, mass_settings.clavicle_mass_percentage, ctx.skeleton_structure.right_clavicle_joint);
        optional_build_kinematic_target(profile->arm_limb_details, mass_settings.arm_mass_percentage, ctx.skeleton_structure.right_arm_joint);
        optional_build_kinematic_target(profile->forearm_limb_details, mass_settings.forearm_mass_percentage, ctx.skeleton_structure.right_forearm_joint);
        optional_build_kinematic_target(profile->hand_limb_details, mass_settings.hand_mass_percentage, ctx.skeleton_structure.right_hand_joint);

        optional_build_kinematic_target(profile->leg_limb_details, mass_settings.leg_mass_percentage, ctx.skeleton_structure.left_leg_joint);
        optional_build_kinematic_target(profile->calf_limb_details, mass_settings.calf_mass_percentage, ctx.skeleton_structure.left_calf_joint);
        optional_build_kinematic_target(profile->foot_limb_details, mass_settings.foot_mass_percentage, ctx.skeleton_structure.left_foot_joint);

        optional_build_kinematic_target(profile->leg_limb_details, mass_settings.leg_mass_percentage, ctx.skeleton_structure.right_leg_joint);
        optional_build_kinematic_target(profile->calf_limb_details, mass_settings.calf_mass_percentage, ctx.skeleton_structure.right_calf_joint);
        optional_build_kinematic_target(profile->foot_limb_details, mass_settings.foot_mass_percentage, ctx.skeleton_structure.right_foot_joint);

        return true;
    }

    bool PhysicalRagdollBuilder::build_constraint_joints(const RuntimeContext& ctx)
    {
        RagdollSkeletonStructure& structure = ctx.skeleton_structure;

        const trs& head_capsule_bottom_transform = structure.head_joint.constraint_object_space_transform.value();
        const trs& neck_capsule_bottom_transform = structure.neck_joint.constraint_object_space_transform.value();

        const trs& left_clavicle_joint_transform = structure.left_clavicle_joint.joint_object_space_transform;
        const trs& left_clavicle_capsule_bottom_transform = structure.left_clavicle_joint.constraint_object_space_transform.value();
        const trs& left_hand_capsule_bottom_transform = structure.left_hand_joint.constraint_object_space_transform.value();
        const trs& left_hand_joint_transform = structure.left_hand_joint.joint_object_space_transform;

        const trs& right_clavicle_joint_transform = structure.right_clavicle_joint.joint_object_space_transform;
        const trs& right_clavicle_capsule_bottom_transform = structure.right_clavicle_joint.constraint_object_space_transform.value();
        const trs& right_hand_capsule_bottom_transform = structure.right_hand_joint.constraint_object_space_transform.value();
        const trs& right_hand_joint_transform = structure.right_hand_joint.joint_object_space_transform;

        const trs& left_foot_capsule_bottom_transform = structure.left_foot_joint.constraint_object_space_transform.value();

        const trs& right_foot_capsule_bottom_transform = structure.right_foot_joint.constraint_object_space_transform.value();

        vec3 body_forward = (structure.left_foot_end_joint.joint_object_space_transform.position - structure.left_foot_joint.joint_object_space_transform.position) +
                            (structure.right_foot_end_joint.joint_object_space_transform.position - structure.right_foot_joint.joint_object_space_transform.position);
        body_forward.y = 0.0f;
        body_forward = normalize(body_forward);

        // Twist along the bone, positive swing Y bends the bone towards flexion_direction (forward_angle), negative swing Y
        // away from it (back_angle).
        auto create_flexion_joint = [&ctx](RagdollSkeletonStructure::JointStructure& parent_joint,
                                        RagdollSkeletonStructure::JointStructure& child_joint,
                                        const RagdollSkeletonStructure::JointStructure& child_end_joint,
                                        const vec3& flexion_direction,
                                        float forward_angle,
                                        float back_angle,
                                        float twist_angle,
                                        float side_angle)
        {
            const vec3 pivot_position = child_joint.joint_object_space_transform.position;
            const vec3 bone_direction = normalize(child_end_joint.joint_object_space_transform.position - pivot_position);

            const quat bone_frame = make_bone_frame_rotation(bone_direction, -flexion_direction);
            const quat limit_frame = normalize(bone_frame * quat(vec3::up, (forward_angle - back_angle) / 2.0f));

            create_d6_joint(
                ctx.enable_physical_animation,
                ctx.ragdoll,
                parent_joint,
                child_joint,
                trs(pivot_position, limit_frame, vec3(1.0f)),
                trs(pivot_position, bone_frame, vec3(1.0f)),
                -twist_angle, twist_angle,
                (forward_angle + back_angle) / 2.0f, side_angle);
        };

        // Neck -> head
        create_d6_joint(
            ctx.enable_physical_animation,
            ctx.ragdoll,
            structure.neck_joint,
            structure.head_joint,
            head_capsule_bottom_transform,
            head_capsule_bottom_transform,
            deg2rad(-30.0f), deg2rad(30.0f),
            deg2rad(28.0f), deg2rad(16.0f));

        // Body upper -> neck
        create_d6_joint(
            ctx.enable_physical_animation,
            ctx.ragdoll,
            structure.thorax_joint,
            structure.neck_joint,
            neck_capsule_bottom_transform,
            neck_capsule_bottom_transform,
            deg2rad(-30.0f), deg2rad(30.0f),
            deg2rad(35.0f), deg2rad(35.0f));

        // Body middle -> body upper
        create_flexion_joint(structure.abdomen_joint, structure.thorax_joint, structure.neck_joint, body_forward,
            deg2rad(25.0f), deg2rad(15.0f), deg2rad(15.0f), deg2rad(15.0f));

        // Body lower -> body middle
        create_flexion_joint(structure.pelvis_joint, structure.abdomen_joint, structure.thorax_joint, body_forward,
            deg2rad(25.0f), deg2rad(20.0f), deg2rad(25.0f), deg2rad(15.0f));

        // Body upper -> left clavicle
        constexpr float clavicle_d6_swing_y_deg = deg2rad(40.0f);
        create_d6_joint(
            ctx.enable_physical_animation,
            ctx.ragdoll,
            structure.thorax_joint,
            structure.left_clavicle_joint,
            left_clavicle_joint_transform,
            left_clavicle_joint_transform,
            deg2rad(-35.0f), deg2rad(35.0f),
            clavicle_d6_swing_y_deg, deg2rad(30.0f));

        // Clavicle -> arm: twist along the bone, the swing cone is centered down and forward from the horizontal,
        // so it covers hanging arms, the T-pose and reaching forward.
        constexpr float arm_cone_down_angle = deg2rad(45.0f);
        constexpr float arm_cone_forward_angle = deg2rad(30.0f);
        constexpr float arm_swing_angle = deg2rad(85.0f);

        auto create_arm_joint = [&ctx, &body_forward](RagdollSkeletonStructure::JointStructure& clavicle_joint,
                                    RagdollSkeletonStructure::JointStructure& arm_joint,
                                    const RagdollSkeletonStructure::JointStructure& forearm_joint)
        {
            const vec3 shoulder_position = arm_joint.joint_object_space_transform.position;
            const vec3 bone_direction = normalize(forearm_joint.joint_object_space_transform.position - shoulder_position);

            vec3 projected_direction = vec3(bone_direction.x, 0.0f, bone_direction.z);
            projected_direction = length(projected_direction) > 1e-3f ? normalize(projected_direction) : bone_direction;

            const vec3 cone_direction = normalize(
                (projected_direction * cos(arm_cone_forward_angle) + body_forward * sin(arm_cone_forward_angle)) * cos(arm_cone_down_angle) -
                vec3::up * sin(arm_cone_down_angle));

            const quat bone_frame = make_bone_frame_rotation(bone_direction, vec3::up);
            const quat cone_frame = normalize(shortest_arc(bone_direction, cone_direction) * bone_frame);

            create_d6_joint(
                ctx.enable_physical_animation,
                ctx.ragdoll,
                clavicle_joint,
                arm_joint,
                trs(shoulder_position, cone_frame, vec3(1.0f)),
                trs(shoulder_position, bone_frame, vec3(1.0f)),
                deg2rad(-55.0f), deg2rad(55.0f),
                arm_swing_angle, arm_swing_angle);
        };

        // Left clavicle -> left arm
        create_arm_joint(structure.left_clavicle_joint, structure.left_arm_joint, structure.left_forearm_joint);

        // Left arm -> left forearm, the forearm bends forward.
        constexpr float elbow_bend_angle = deg2rad(140.0f);
        constexpr float elbow_over_straightening_angle = deg2rad(10.0f);
        create_flexion_joint(structure.left_arm_joint, structure.left_forearm_joint, structure.left_hand_joint, body_forward,
            elbow_bend_angle, elbow_over_straightening_angle, deg2rad(20.0f), deg2rad(10.0f));

        // Left forearm -> left hand
        create_d6_joint(
            ctx.enable_physical_animation,
            ctx.ragdoll,
            structure.left_forearm_joint,
            structure.left_hand_joint,
            left_hand_joint_transform,
            left_hand_joint_transform,
            deg2rad(-60.0f), deg2rad(60.0f),
            deg2rad(40.0f), deg2rad(58.0f));

        // Body upper -> right clavicle
        create_d6_joint(
            ctx.enable_physical_animation,
            ctx.ragdoll,
            structure.thorax_joint,
            structure.right_clavicle_joint,
            right_clavicle_joint_transform,
            right_clavicle_joint_transform,
            deg2rad(-35.0f), deg2rad(35.0f),
            clavicle_d6_swing_y_deg, deg2rad(30.0f));

        // Right clavicle -> right arm
        create_arm_joint(structure.right_clavicle_joint, structure.right_arm_joint, structure.right_forearm_joint);

        // Right arm -> right forearm
        create_flexion_joint(structure.right_arm_joint, structure.right_forearm_joint, structure.right_hand_joint, body_forward,
            elbow_bend_angle, elbow_over_straightening_angle, deg2rad(20.0f), deg2rad(10.0f));

        // Right forearm -> right hand
        create_d6_joint(
            ctx.enable_physical_animation,
            ctx.ragdoll,
            structure.right_forearm_joint,
            structure.right_hand_joint,
            right_hand_joint_transform,
            right_hand_joint_transform,
            deg2rad(-60.0f), deg2rad(60.0f),
            deg2rad(40.0f), deg2rad(58.0f));

        // Pelvis -> left up leg
        constexpr float up_leg_forward_angle_deg = deg2rad(75.0f);
        constexpr float up_leg_back_angle_deg = deg2rad(30.0f);
        create_flexion_joint(structure.pelvis_joint, structure.left_leg_joint, structure.left_calf_joint, body_forward,
            up_leg_forward_angle_deg, up_leg_back_angle_deg, deg2rad(20.0f), deg2rad(50.0f));

        // Left up leg -> left leg, the shin bends backward.
        constexpr float knee_bend_angle = deg2rad(165.0f);
        constexpr float knee_over_straightening_angle = deg2rad(5.0f);
        create_flexion_joint(structure.left_leg_joint, structure.left_calf_joint, structure.left_foot_joint, -body_forward,
            knee_bend_angle, knee_over_straightening_angle, deg2rad(20.0f), deg2rad(22.0f));

        // Left leg -> left foot
        create_d6_joint(
            ctx.enable_physical_animation,
            ctx.ragdoll,
            structure.left_calf_joint,
            structure.left_foot_joint,
            left_foot_capsule_bottom_transform,
            left_foot_capsule_bottom_transform,
            deg2rad(-27.0f), deg2rad(27.0f),
            deg2rad(35.0f), deg2rad(70.0f));

        // Pelvis -> right up leg
        create_flexion_joint(structure.pelvis_joint, structure.right_leg_joint, structure.right_calf_joint, body_forward,
            up_leg_forward_angle_deg, up_leg_back_angle_deg, deg2rad(20.0f), deg2rad(50.0f));

        // Right up leg -> right leg
        create_flexion_joint(structure.right_leg_joint, structure.right_calf_joint, structure.right_foot_joint, -body_forward,
            knee_bend_angle, knee_over_straightening_angle, deg2rad(20.0f), deg2rad(22.0f));

        // Right leg -> right foot
        create_d6_joint(
            ctx.enable_physical_animation,
            ctx.ragdoll,
            structure.right_calf_joint,
            structure.right_foot_joint,
            right_foot_capsule_bottom_transform,
            right_foot_capsule_bottom_transform,
            deg2rad(-27.0f), deg2rad(27.0f),
            deg2rad(35.0f), deg2rad(70.0f));

        return true;
    }

    bool PhysicalRagdollBuilder::build_drive_joints(const RuntimeContext& ctx)
    {
        if (!ctx.enable_physical_animation)
        {
            return true;
        }

        if (ctx.default_profile == nullptr)
        {
            return false;
        }

        auto optional_build_drive_joint = [&ctx](const PhysicalLimbDetails& limb_details,
                                              RagdollSkeletonStructure::JointStructure& joint)
        {
            if (!limb_details.motor_drive.has_value())
            {
                return;
            }
            create_drive_joint(
                ctx.ragdoll,
                limb_details,
                joint.kinematic_target.value(),
                joint.physical_limb.value());
        };

        optional_build_drive_joint(ctx.default_profile->head_limb_details, ctx.skeleton_structure.head_joint);
        optional_build_drive_joint(ctx.default_profile->neck_limb_details, ctx.skeleton_structure.neck_joint);

        optional_build_drive_joint(ctx.default_profile->body_upper_limb_details, ctx.skeleton_structure.thorax_joint);
        optional_build_drive_joint(ctx.default_profile->body_middle_limb_details, ctx.skeleton_structure.abdomen_joint);
        optional_build_drive_joint(ctx.default_profile->body_lower_limb_details, ctx.skeleton_structure.pelvis_joint);

        optional_build_drive_joint(ctx.default_profile->clavicle_limb_details, ctx.skeleton_structure.left_clavicle_joint);
        optional_build_drive_joint(ctx.default_profile->arm_limb_details, ctx.skeleton_structure.left_arm_joint);
        optional_build_drive_joint(ctx.default_profile->forearm_limb_details, ctx.skeleton_structure.left_forearm_joint);
        optional_build_drive_joint(ctx.default_profile->hand_limb_details, ctx.skeleton_structure.left_hand_joint);

        optional_build_drive_joint(ctx.default_profile->clavicle_limb_details, ctx.skeleton_structure.right_clavicle_joint);
        optional_build_drive_joint(ctx.default_profile->arm_limb_details, ctx.skeleton_structure.right_arm_joint);
        optional_build_drive_joint(ctx.default_profile->forearm_limb_details, ctx.skeleton_structure.right_forearm_joint);
        optional_build_drive_joint(ctx.default_profile->hand_limb_details, ctx.skeleton_structure.right_hand_joint);

        optional_build_drive_joint(ctx.default_profile->leg_limb_details, ctx.skeleton_structure.left_leg_joint);
        optional_build_drive_joint(ctx.default_profile->calf_limb_details, ctx.skeleton_structure.left_calf_joint);
        optional_build_drive_joint(ctx.default_profile->foot_limb_details, ctx.skeleton_structure.left_foot_joint);

        optional_build_drive_joint(ctx.default_profile->leg_limb_details, ctx.skeleton_structure.right_leg_joint);
        optional_build_drive_joint(ctx.default_profile->calf_limb_details, ctx.skeleton_structure.right_calf_joint);
        optional_build_drive_joint(ctx.default_profile->foot_limb_details, ctx.skeleton_structure.right_foot_joint);

        return true;
    }

    // Body inertia data in world space.
    struct BodyInertia
    {
        float mass = 0.0f;
        physx::PxVec3 com = physx::PxVec3(0.0f);
        physx::PxQuat principal_axes = physx::PxQuat(physx::PxIdentity);
        physx::PxVec3 principal_inertia = physx::PxVec3(0.0f);
    };

    static BodyInertia get_body_inertia(const physx::PxRigidDynamic* rigid_dynamic)
    {
        using namespace physx;

        const PxTransform global_pose = rigid_dynamic->getGlobalPose();
        const PxTransform cmass_pose = rigid_dynamic->getCMassLocalPose();

        BodyInertia result;
        result.mass = rigid_dynamic->getMass();
        result.com = global_pose.transform(cmass_pose.p);
        result.principal_axes = global_pose.q * cmass_pose.q;
        result.principal_inertia = rigid_dynamic->getMassSpaceInertiaTensor();
        return result;
    }

    // Inertia of a body about the axis n passing through point.
    static float get_inertia_about_axis(const BodyInertia& body, const physx::PxVec3& point, const physx::PxVec3& n)
    {
        using namespace physx;

        const PxVec3 local_n = body.principal_axes.rotateInv(n);
        const float inertia_cm = body.principal_inertia.x * local_n.x * local_n.x +
                                 body.principal_inertia.y * local_n.y * local_n.y +
                                 body.principal_inertia.z * local_n.z * local_n.z;

        const PxVec3 offset = body.com - point;
        const float distance_sq = std::max(offset.magnitudeSquared() - offset.dot(n) * offset.dot(n), 0.0f);

        return inertia_cm + body.mass * distance_sq;
    }

    static RagdollLimbComponent* get_ragdoll_limb_component(Entity limb)
    {
        if (PhysicalAnimationLimbComponent* physical_animation_limb = limb.get_component_if_exists<PhysicalAnimationLimbComponent>())
        {
            return physical_animation_limb;
        }
        return limb.get_component_if_exists<RagdollLimbComponent>();
    }

    bool PhysicalRagdollBuilder::try_finalize_ragdoll(RagdollComponent* ragdoll_component)
    {
        ASSERT(ragdoll_component != nullptr);

        if (ragdoll_component->loaded)
        {
            return true;
        }

        if (!are_bodies_created(ragdoll_component))
        {
            return false;
        }

        apply_constraint_limits(ragdoll_component);
        apply_pelvis_attachment(ragdoll_component);

        ragdoll_component->loaded = true;
        return true;
    }

    bool PhysicalRagdollBuilder::are_bodies_created(const RagdollComponent* ragdoll_component)
    {
        if (ragdoll_component->limbs.empty())
        {
            return false;
        }

        for (const EntityPtr& limb_ptr : ragdoll_component->limbs)
        {
            Entity limb = limb_ptr.get();

            const ShapeComponent* shape_component = ShapeUtils::get_shape_component(limb);
            const DynamicBodyComponent* body_component = limb.get_component_if_exists<DynamicBodyComponent>();

            if (body_component == nullptr || shape_component == nullptr)
            {
                return false;
            }

            const physx::PxRigidActor* actor = body_component->get_rigid_actor();
            const physx::PxShape* shape = shape_component->get_shape();

            if (actor == nullptr || actor->userData == nullptr ||
                shape == nullptr || shape->userData == nullptr)
            {
                return false;
            }
        }

        return true;
    }

    void PhysicalRagdollBuilder::apply_constraint_limits(const RagdollComponent* ragdoll_component)
    {
        using namespace physx;

        const RagdollSettings::ConstraintLimitSettings& settings = ragdoll_component->settings.constraint_limit_settings;

        struct LimbNode
        {
            Entity limb;
            Entity::Handle parent = Entity::null_handle;
            D6JointComponent* parent_joint = nullptr;
            const PxRigidDynamic* rigid_dynamic = nullptr;
            BodyInertia inertia;
        };

        std::vector<LimbNode> nodes;
        nodes.reserve(ragdoll_component->limbs.size());

        for (const EntityPtr& limb_ptr : ragdoll_component->limbs)
        {
            Entity limb = limb_ptr.get();

            LimbNode& node = nodes.emplace_back();
            node.limb = limb;

            if (const DynamicBodyComponent* body_component = limb.get_component_if_exists<DynamicBodyComponent>())
            {
                node.rigid_dynamic = body_component->get_rigid_dynamic();
            }

            if (RagdollLimbComponent* limb_component = get_ragdoll_limb_component(limb))
            {
                node.parent_joint = dynamic_cast<D6JointComponent*>(limb_component->parent_joint_component.get_for_write());
                if (node.parent_joint != nullptr)
                {
                    node.parent = node.parent_joint->get_first_entity_ptr().get().get_handle();
                }
            }
        }

        auto is_in_subtree = [&nodes](Entity::Handle handle, Entity::Handle subtree_root)
        {
            while (handle != Entity::null_handle)
            {
                if (handle == subtree_root)
                {
                    return true;
                }

                auto node_iter = std::find_if(nodes.begin(), nodes.end(), [handle](const LimbNode& node)
                    {
                        return node.limb.get_handle() == handle;
                    });

                if (node_iter == nodes.end())
                {
                    return false;
                }
                handle = node_iter->parent;
            }
            return false;
        };

        const float omega = 2.0f * M_PI * std::max(settings.frequency, 0.0f);

        struct LimitValues
        {
            D6JointComponent* joint = nullptr;
            float stiffness = 0.0f;
            float damping = 0.0f;
        };
        std::vector<LimitValues> limit_values;

        PhysicsEngine::execute_read([&]()
            {
                for (LimbNode& node : nodes)
                {
                    if (node.rigid_dynamic != nullptr)
                    {
                        node.inertia = get_body_inertia(node.rigid_dynamic);
                    }
                }

                for (const LimbNode& node : nodes)
                {
                    if (node.parent_joint == nullptr || node.rigid_dynamic == nullptr)
                    {
                        continue;
                    }

                    // Joint frame of the constraint in world space (actor1 = this limb).
                    const PxTransform joint_world_frame = node.rigid_dynamic->getGlobalPose() * create_PxTransform(node.parent_joint->get_second_local_frame());

                    // Inertia of everything below the joint about the joint, worst of the three joint axes.
                    float subtree_inertia = 0.0f;
                    for (uint32 axis = 0; axis < 3; ++axis)
                    {
                        PxVec3 unit_axis = PxVec3(0.0f);
                        unit_axis[axis] = 1.0f;
                        const PxVec3 n = joint_world_frame.q.rotate(unit_axis);

                        float axis_inertia = 0.0f;
                        for (const LimbNode& other : nodes)
                        {
                            if (other.rigid_dynamic != nullptr && is_in_subtree(other.limb.get_handle(), node.limb.get_handle()))
                            {
                                axis_inertia += get_inertia_about_axis(other.inertia, joint_world_frame.p, n);
                            }
                        }

                        subtree_inertia = std::max(subtree_inertia, axis_inertia);
                    }

                    if (subtree_inertia <= 0.0f)
                    {
                        continue;
                    }

                    LimitValues& values = limit_values.emplace_back();
                    values.joint = node.parent_joint;
                    values.stiffness = subtree_inertia * omega * omega;
                    values.damping = 2.0f * settings.damping_ratio * std::sqrt(values.stiffness * subtree_inertia);
                }
            });

        for (const LimitValues& values : limit_values)
        {
            values.joint->twist_limit_stiffness = max(values.stiffness, 120.0f);
            values.joint->twist_limit_damping = max(values.damping, 20.0f);
        }
    }

    void PhysicalRagdollBuilder::apply_pelvis_attachment(RagdollComponent* ragdoll_component)
    {
        PhysicalAnimationComponent* physical_animation_component = dynamic_cast<PhysicalAnimationComponent*>(ragdoll_component);
        if (physical_animation_component == nullptr ||
            physical_animation_component->use_fixed_pelvis_attachment ||
            physical_animation_component->attachment_body.is_empty())
        {
            return;
        }

        DistanceJointComponent* joint_component = physical_animation_component->attachment_body.get().get_component_if_exists<DistanceJointComponent>();
        if (joint_component == nullptr)
        {
            return;
        }

        const DynamicBodyComponent* pelvis_body = joint_component->get_second_entity_ptr().get().get_component_if_exists<DynamicBodyComponent>();
        if (pelvis_body == nullptr)
        {
            return;
        }

        RagdollSettings::PelvisAttachmentSettings settings = ragdoll_component->settings.pelvis_attachment_settings;

        // Distance joint springs are force based.
        const float mass = pelvis_body->mass.get();
        const float omega = 2.0f * M_PI * settings.frequency;
        const float stiffness = mass * omega * omega;

        physical_animation_component->pelvis_attachment_stiffness = stiffness;
        physical_animation_component->pelvis_attachment_damping = 2.0f * settings.damping_ratio * std::sqrt(stiffness * mass);

        joint_component->stiffness = physical_animation_component->pelvis_attachment_stiffness;
        joint_component->damping = physical_animation_component->pelvis_attachment_damping;
        joint_component->max_distance = settings.max_distance;
        joint_component->min_distance.get_for_write() = 0.0f;
    }
}
