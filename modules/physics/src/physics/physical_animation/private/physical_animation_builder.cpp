#include "physics/physical_animation/physical_animation_builder.h"
#include "physics/core/physics.h"
#include "physics/body_component.h"
#include "physics/shape_component.h"
#include "physics/core/physics_utils.h"
#include "physics/shape_utils.h"
#include "physics/physical_animation/physical_animation_component.h"
#include "physics/aggregate_holder_component.h"
#include "physics/joint.h"

#include <core/math.h>
#include "core/debug/debug_var.h"

#include <ecs/base_components/transform_component.h>
#include <ecs/world.h>

namespace era_engine::physics
{
	static Entity create_child_entity(Entity parent, 
		std::string_view entity_name,
		const PhysicalLimbDetails& details,
		uint32 joint_id, 
		RagdollLimbType type,
		const ref<PhysicsLimbChain>& chain)
	{
		Entity child = parent.get_world()->create_entity(entity_name.data());
		child.set_parent(parent.get_handle());

		PhysicalAnimationLimbComponent* limb_component = child.add_component<PhysicalAnimationLimbComponent>();
		limb_component->joint_id = joint_id;
		limb_component->type = type;
		if (details.motor_drive.has_value())
		{
			const MotorDriveDetails& motor_drive = details.motor_drive.value();

			limb_component->angular_range = motor_drive.angular_range;
			limb_component->linear_range = motor_drive.linear_range;
			limb_component->angular_damping_range = motor_drive.angular_damping_range;
			limb_component->linear_damping_range = motor_drive.linear_damping_range;
		}
		
		limb_component->ragdoll_ptr = EntityPtr{ parent };

		PhysicalAnimationComponent* physical_animation_component = parent.get_component<PhysicalAnimationComponent>();
		physical_animation_component->limbs.push_back(EntityPtr{ child });

		chain->connected_limbs.push_back(EntityPtr{ child });

		return child;
	}

	static DynamicBodyComponent* create_dynamic_body(
		Entity& entity,
		const float mass,
		const float max_contact_impulse = 400)
	{
		DynamicBodyComponent* dynamic_body_component = entity.add_component<DynamicBodyComponent>();
		dynamic_body_component->mass.get_for_write() = mass;
		dynamic_body_component->ccd.get_for_write() = true;
		dynamic_body_component->use_gravity.get_for_write() = false;
		dynamic_body_component->simulated.get_for_write() = false;
		dynamic_body_component->linear_damping.get_for_write() = 0.15f;
		dynamic_body_component->angular_damping.get_for_write() = 0.25f;
		dynamic_body_component->max_contact_impulse.get_for_write() = max_contact_impulse;
		dynamic_body_component->solver_position_iterations_count.get_for_write() = 16;
		dynamic_body_component->solver_velocity_iterations_count.get_for_write() = 4;
		dynamic_body_component->sleep_threshold.get_for_write() = 0.01f;
		dynamic_body_component->stabilization_threshold.get_for_write() = 0.1f;

		return dynamic_body_component;
	}

	static Entity create_attachment_dynamic_body(
		Entity entity,
		std::string_view entity_name,
		const float mass)
	{
		ASSERT(entity.is_valid());

		Entity attachment = entity.get_world()->create_entity(entity_name.data());
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

	static void create_collision_joint(
		const Entity& source,
		Entity& e0,
		Entity& e1)
	{
		JointComponent::BaseDescriptor descriptor;
		descriptor.connected_entity = e0.get_data_weakref();
		descriptor.second_connected_entity = e1.get_data_weakref();
		descriptor.local_frame = trs::identity;
		descriptor.second_local_frame = trs::identity;

		Entity joint_entity = e0.get_world()->create_entity();
		joint_entity.set_parent(source.get_handle());

		D6JointComponent* joint_component = joint_entity.add_component<D6JointComponent>(descriptor);

		joint_component->disable_preprocessing = true;
		joint_component->enable_collision.get_for_write() = false;

		joint_component->linear_x_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
		joint_component->linear_y_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
		joint_component->linear_z_motion_type.get_for_write() = D6JointComponent::Motion::FREE;

		joint_component->twist_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
		joint_component->swing_y_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
		joint_component->swing_z_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
	}

	static void create_d6_joint(
		const Entity& source,
		Entity& e0, 
		Entity& e1,
		const trs& e0_joint_transform,
		const trs& e1_joint_transform,
		const trs& e0_local_transform,
		const trs& e1_local_transform,
		float twist_min_deg,
		float twist_max_deg,
		float swing_y_deg,
		float swing_z_deg)
	{
		trs e0_to_e0_joint_transform = invert(e0_local_transform) * e0_joint_transform;
		e0_to_e0_joint_transform.rotation = normalize(e0_to_e0_joint_transform.rotation);
		e0_to_e0_joint_transform.scale = vec3(1.0f);

		trs e1_to_e1_joint_transform = invert(e1_local_transform) * e1_joint_transform;
		e1_to_e1_joint_transform.rotation = normalize(e1_to_e1_joint_transform.rotation);
		e1_to_e1_joint_transform.scale = vec3(1.0f);

		JointComponent::BaseDescriptor descriptor;
		descriptor.connected_entity = e0.get_data_weakref();
		descriptor.second_connected_entity = e1.get_data_weakref();
		descriptor.local_frame = e0_to_e0_joint_transform;
		descriptor.second_local_frame = e1_to_e1_joint_transform;

		Entity joint_entity = e0.get_world()->create_entity();
		joint_entity.set_parent(source.get_handle());

		e0.get_component<PhysicalAnimationLimbComponent>()->joint_entity_ptr = EntityPtr{ joint_entity };

		D6JointComponent* joint_component = joint_entity.add_component<D6JointComponent>(descriptor);

		joint_component->enable_collision.get_for_write() = false;

		joint_component->linear_x_motion_type.get_for_write() = D6JointComponent::Motion::LOCKED;
		joint_component->linear_y_motion_type.get_for_write() = D6JointComponent::Motion::LOCKED;
		joint_component->linear_z_motion_type.get_for_write() = D6JointComponent::Motion::LOCKED;

		e1.get_component<PhysicalAnimationLimbComponent>()->parent_joint_component = ComponentPtr{ joint_component };

		if (fuzzy_equals(twist_max_deg, 180.0f) && fuzzy_equals(twist_min_deg, -180.0f))
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
			joint_component->twist_min_limit.get_for_write() = deg2rad(twist_min_deg);
			joint_component->twist_max_limit.get_for_write() = deg2rad(twist_max_deg);

			joint_component->twist_limit_damping.get_for_write() = 10.0f;
			joint_component->twist_limit_stiffness.get_for_write() = 100.0f;
			joint_component->twist_limit_restitution.get_for_write() = 0.0f;
		}

		bool any_moving_swing = false;

		if (fuzzy_equals(swing_y_deg, 180.0f))
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
			joint_component->swing_y_limit.get_for_write() = deg2rad(swing_y_deg);
			any_moving_swing = true;
		}

		if (fuzzy_equals(swing_z_deg, 180.0f))
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
			joint_component->swing_z_limit.get_for_write() = deg2rad(swing_z_deg);
			any_moving_swing = true;
		}

		if (any_moving_swing)
		{
			joint_component->swing_limit_damping.get_for_write() = 10.0f;
			joint_component->swing_limit_stiffness.get_for_write() = 100.0f;
			joint_component->swing_limit_restitution.get_for_write() = 0.0f;
		}
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

		const MotorDriveDetails& motor_drive = details.motor_drive.value();

		JointComponent::BaseDescriptor descriptor;
		descriptor.connected_entity = e0.get_data_weakref();
		descriptor.second_connected_entity = e1.get_data_weakref();
		descriptor.local_frame = trs::identity;
		descriptor.second_local_frame = trs::identity;

		Entity joint_entity = e1.get_world()->create_entity();
		joint_entity.set_parent(source.get_handle());

		e1.get_component<PhysicalAnimationLimbComponent>()->drive_joint_entity_ptr = EntityPtr{ joint_entity };

		D6JointComponent* joint_component = joint_entity.add_component<D6JointComponent>(descriptor);

		joint_component->perform_slerp_drive = motor_drive.enable_slerp_drive;
		joint_component->disable_preprocessing = true;

		joint_component->enable_collision.get_for_write() = false;
		joint_component->drive_limits_are_forces.get_for_write() = true;

		joint_component->linear_x_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
		joint_component->linear_y_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
		joint_component->linear_z_motion_type.get_for_write() = D6JointComponent::Motion::FREE;

		joint_component->twist_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
		joint_component->swing_y_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
		joint_component->swing_z_motion_type.get_for_write() = D6JointComponent::Motion::FREE;

		joint_component->linear_drive_stiffness = motor_drive.linear_drive_stiffness * RagdollStrengthConfig::LINEAR_STIFFNESS_MODIFIER;
		joint_component->linear_drive_damping = motor_drive.linear_damping_range.y * RagdollStrengthConfig::LINEAR_DAMPING_MODIFIER;
		joint_component->linear_drive_force_limit = motor_drive.max_force;
		joint_component->linear_drive_accelerated = motor_drive.accelerated;

		e1.get_component<PhysicalAnimationLimbComponent>()->drive_joint_component = ComponentPtr{ joint_component };

		if (motor_drive.enable_slerp_drive)
		{
			joint_component->slerp_drive_damping.get_for_write() = motor_drive.angular_damping_range.y * RagdollStrengthConfig::ANGULAR_DAMPING_MODIFIER;
			joint_component->slerp_drive_force_limit.get_for_write() = motor_drive.max_force;
			joint_component->slerp_drive_stiffness.get_for_write() = motor_drive.angular_drive_stiffness * RagdollStrengthConfig::ANGULAR_STIFFNESS_MODIFIER;
			joint_component->slerp_drive_accelerated.get_for_write() = motor_drive.accelerated;
		}
		else
		{
			joint_component->swing_drive_force_limit.get_for_write() = motor_drive.max_force;
			joint_component->swing_drive_accelerated.get_for_write() = motor_drive.accelerated;
			joint_component->swing_drive_stiffness.get_for_write() = motor_drive.angular_drive_stiffness * RagdollStrengthConfig::ANGULAR_STIFFNESS_MODIFIER;
			joint_component->swing_drive_damping.get_for_write() = motor_drive.angular_damping_range.y * RagdollStrengthConfig::ANGULAR_DAMPING_MODIFIER;

			joint_component->twist_drive_stiffness.get_for_write() = motor_drive.angular_drive_stiffness * RagdollStrengthConfig::ANGULAR_STIFFNESS_MODIFIER;
			joint_component->twist_drive_damping.get_for_write() = motor_drive.angular_damping_range.y * RagdollStrengthConfig::ANGULAR_DAMPING_MODIFIER;
			joint_component->twist_drive_force_limit.get_for_write() = motor_drive.max_force;
			joint_component->twist_drive_accelerated.get_for_write() = motor_drive.accelerated;
		}
	}

	// Return bottom of the box, used for creating d6s
	// Bottom = lowest point along length axis.
	static trs position_box_between_joints(
		BoxShapeComponent* box_shape_component,
		const vec3& size,
		const trs& from_joint_transform,
		const trs& to_joint_transform,
		const trs& owner_joint_transform,
		const vec3& local_position_adjustment = vec3::zero)
	{
		box_shape_component->half_extents = (size / 2.0f) / owner_joint_transform.scale.x;

		ASSERT(squared_length(box_shape_component->half_extents) > 0.0f);

		const vec3 offset = to_joint_transform.position - from_joint_transform.position;
		const vec3 center_between_joints = from_joint_transform.position + offset / 2.0f;
		const vec3 direction = normalize(offset);

		const trs box_transform(
			center_between_joints,
			shortest_arc(vec3(1.0f, 0.0f, 0.0f), direction),
			vec3(1.0f));

		const trs box_transform_relative_to_owner = invert(owner_joint_transform) *  box_transform;

		box_shape_component->local_position.get_for_write() = box_transform_relative_to_owner.position + local_position_adjustment;
		box_shape_component->local_rotation.get_for_write() = box_transform_relative_to_owner.rotation;

		return trs(box_transform.position - direction * size.x / 2.0f, box_transform.rotation, box_transform.scale);
	}

	// Return bottom of the capsule, used for creating d6s
	// Bottom = lowest point on x axis.
	static trs position_capsule_between_joints_from_radius(
		CapsuleShapeComponent* capsule_shape_component,
		const float radius,
		const trs& from_joint_transform,
		const trs& to_joint_transform,
		const trs& owner_joint_transform,
		const float height_multiplier,
		const vec3& local_position_adjustment = vec3::zero)
	{
		const vec3 offset = to_joint_transform.position - from_joint_transform.position;
		const float height = (length(offset) - 2 * radius) * height_multiplier;

		capsule_shape_component->radius = radius / owner_joint_transform.scale.x;
		capsule_shape_component->half_height = (height / 2.0f) / owner_joint_transform.scale.x;

		ASSERT(capsule_shape_component->radius > 0.0f);
		ASSERT(capsule_shape_component->half_height > 0.0f);

		const vec3 center_between_joints = from_joint_transform.position + offset / 2.0f;
		const vec3 direction = normalize(offset);

		const trs capsule_transform(
			center_between_joints,
			shortest_arc(vec3(1.0f, 0.0f, 0.0f), direction),
			vec3(1.0f));

		const trs capsule_transform_relative_to_owner = invert(owner_joint_transform) * capsule_transform;

		capsule_shape_component->local_position.get_for_write() = capsule_transform_relative_to_owner.position + local_position_adjustment;
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
		const vec3& offset_for_from_joint = vec3::zero,
		const vec3& local_position_adjustment = vec3::zero)
	{
		const vec3 offset = to_joint_transform.position - from_joint_transform.position;

		capsule_shape_component->radius = radius / owner_joint_transform.scale.x;
		capsule_shape_component->half_height = half_height / owner_joint_transform.scale.x;

		ASSERT(capsule_shape_component->radius > 0.0f);
		ASSERT(capsule_shape_component->half_height > 0.0f);

		const vec3 center_between_joints = from_joint_transform.position + offset / 2.0f + offset_for_from_joint;
		const vec3 direction = normalize(offset);

		const trs capsule_transform(
			center_between_joints,
			shortest_arc(vec3(1.0f, 0.0f, 0.0f), direction),
			vec3(1.0f));

		const trs capsule_transform_relative_to_owner = invert(owner_joint_transform) * capsule_transform;

		capsule_shape_component->local_position.get_for_write() = capsule_transform_relative_to_owner.position + local_position_adjustment;
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
		const vec3& local_position_adjustment = vec3::zero)
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
			center_between_joints,
			shortest_arc(vec3(0.0f, 1.0f, 0.0f), direction),
			vec3(1.0f));
		const trs capsule_transform_relative_to_owner = invert(owner_joint_transform) * capsule_transform;

		capsule_shape_component->local_position.get_for_write() = capsule_transform_relative_to_owner.position + local_position_adjustment;
		capsule_shape_component->local_rotation.get_for_write() = capsule_transform_relative_to_owner.rotation;

		const vec3 capsule_x_axis = capsule_transform.rotation * vec3(1.0f, 0.0f, 0.0f);
		return trs(capsule_transform.position - direction * radius, capsule_transform.rotation, capsule_transform.scale) * trs(vec3::zero, shortest_arc(capsule_x_axis, direction), vec3(1.0f));
	}

	void PhysicalAnimationBuilderUtils::build_simulated_body(Entity ragdoll)
	{
		using namespace animation;

		SkeletonComponent* skeleton_component = ragdoll.get_component<SkeletonComponent>();

		const Skeleton* skeleton = skeleton_component->skeleton.get();
		if (skeleton == nullptr)
		{
			ASSERT(false);
			return;
		}

		PhysicalAnimationComponent* physical_animation_component = ragdoll.get_component<PhysicalAnimationComponent>();
		const RagdollSettings& settings = physical_animation_component->settings;
		const RagdollJointIds& joint_init_ids = physical_animation_component->joint_init_ids;

		AggregateHolderComponent* aggregate_component = ragdoll.add_component<AggregateHolderComponent>();
		aggregate_component->enable_self_collision = true;
		aggregate_component->max_actors = 48;

		ref<RagdollProfile> ragdoll_profile = physical_animation_component->get_ragdoll_profile();

		const trs& ragdoll_world_transform = ragdoll.get_component<TransformComponent>()->get_world_transform();

		const PhysicalLimbDetails& head_limb_details = ragdoll_profile->head_limb_details;
		const PhysicalLimbDetails& neck_limb_details = ragdoll_profile->neck_limb_details;
		const PhysicalLimbDetails& body_upper_limb_details = ragdoll_profile->body_upper_limb_details;
		const PhysicalLimbDetails& body_middle_limb_details = ragdoll_profile->body_middle_limb_details;
		const PhysicalLimbDetails& clavicle_limb_details = ragdoll_profile->clavicle_limb_details;
		const PhysicalLimbDetails& arm_limb_details = ragdoll_profile->arm_limb_details;
		const PhysicalLimbDetails& forearm_limb_details = ragdoll_profile->forearm_limb_details;
		const PhysicalLimbDetails& hand_limb_details = ragdoll_profile->hand_limb_details;
		const PhysicalLimbDetails& leg_limb_details = ragdoll_profile->leg_limb_details;
		const PhysicalLimbDetails& calf_limb_details = ragdoll_profile->calf_limb_details;
		const PhysicalLimbDetails& foot_limb_details = ragdoll_profile->foot_limb_details;

		physical_animation_component->neck_chain = make_ref<PhysicsLimbChain>();
		physical_animation_component->body_chain = make_ref<PhysicsLimbChain>();

		physical_animation_component->left_arm_chain = make_ref<PhysicsLimbChain>();
		physical_animation_component->right_arm_chain = make_ref<PhysicsLimbChain>();

		physical_animation_component->left_leg_chain = make_ref<PhysicsLimbChain>();
		physical_animation_component->right_leg_chain = make_ref<PhysicsLimbChain>();

		const float mass = physical_animation_component->mass;

		const trs neck_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.neck_idx);

		trs head_end_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.head_end_idx);
		head_end_joint_transform.position = head_end_joint_transform.position + head_end_joint_transform.rotation * settings.head_end_joint_adjastment;

		const trs head_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.head_idx);

		const trs thorax_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.spine_03_idx);
		const trs abdomen_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.spine_01_idx);
		const trs pelvis_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.pelvis_idx);

		const trs left_clavicle_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.clavicle_l_idx);
		const trs left_arm_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.upperarm_l_idx);
		const trs left_forearm_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.lowerarm_l_idx);
		const trs left_hand_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.hand_l_idx);
		const trs left_hand_end_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.hand_end_l_idx);

		const trs right_clavicle_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.clavicle_r_idx);
		const trs right_arm_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.upperarm_r_idx);
		const trs right_forearm_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.lowerarm_r_idx);
		const trs right_hand_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.hand_r_idx);
		const trs right_hand_end_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.hand_end_r_idx);

		const trs left_up_leg_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.thigh_l_idx);
		const trs left_leg_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.calf_l_idx);
		const trs left_foot_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.foot_l_idx);
		const trs left_foot_end_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.foot_end_l_idx);

		const trs right_up_leg_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.thigh_r_idx);
		const trs right_leg_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.calf_r_idx);
		const trs right_foot_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.foot_r_idx);
		const trs right_foot_end_joint_transform = SkeletonUtils::get_object_space_joint_transform(skeleton, joint_init_ids.foot_end_r_idx);

		// Lower body goes from pelvis to middle, and upper body goes from middle to thorax
		const trs middle_abdomen_thorax_transform = trs(
			lerp(abdomen_joint_transform.position, thorax_joint_transform.position, 0.5f),
			slerp(abdomen_joint_transform.rotation, thorax_joint_transform.rotation, 0.5f),
			lerp(abdomen_joint_transform.scale, thorax_joint_transform.scale, 0.5f));

		const float distance_between_arms = length(left_arm_joint_transform.position - right_arm_joint_transform.position);
		const float distance_between_hand_and_hand_end = length(left_hand_joint_transform.position - left_hand_end_joint_transform.position);

		const float distance_between_foot_and_foot_end = length(left_foot_joint_transform.position - left_foot_end_joint_transform.position);
		const float distance_between_foot_y_and_foot_end_y = abs(left_foot_joint_transform.position.y - left_foot_end_joint_transform.position.y);

		ref<PhysicsMaterial> material = PhysicsHolder::physics_ref->create_material(0.2f, 0.8f, 0.8f);
		ASSERT(material != nullptr);

		Entity head;
		Entity head_attachment;
		trs head_capsule_bottom_transform;

		Entity neck;
		Entity neck_attachment;
		trs neck_capsule_bottom_transform;

		Entity body_upper;
		Entity body_upper_attachment;
		trs body_upper_capsule_bottom_transform;

		Entity body_middle;
		Entity body_middle_attachment;
		trs middle_default_transform;

		Entity body_lower;
		trs lower_default_transform;

		Entity left_clavicle;
		Entity left_clavicle_attachment;
		trs left_clavicle_capsule_bottom_transform;

		Entity left_arm;
		Entity left_arm_attachment;
		trs left_arm_capsule_bottom_transform;

		Entity left_forearm;
		Entity left_forearm_attachment;
		trs left_forearm_capsule_bottom_transform;

		Entity left_hand;
		Entity left_hand_attachment;
		trs left_hand_box_bottom_transform;

		Entity right_clavicle;
		Entity right_clavicle_attachment;
		trs right_clavicle_capsule_bottom_transform;

		Entity right_arm;
		Entity right_arm_attachment;
		trs right_arm_capsule_bottom_transform;

		Entity right_forearm;
		Entity right_forearm_attachment;
		trs right_forearm_capsule_bottom_transform;

		Entity right_hand;
		Entity right_hand_attachment;
		trs right_hand_box_bottom_transform;

		Entity left_up_leg;
		Entity left_up_leg_attachment;
		trs left_up_leg_capsule_bottom_transform;

		Entity left_leg;
		Entity left_leg_attachment;
		trs left_leg_capsule_bottom_transform;

		Entity left_foot;
		Entity left_foot_attachment;
		trs left_foot_box_bottom_transform;

		Entity right_up_leg;
		Entity right_up_leg_attachment;
		trs right_up_leg_capsule_bottom_transform;

		Entity right_leg;
		Entity right_leg_attachment;
		trs right_leg_capsule_bottom_transform;

		Entity right_foot;
		Entity right_foot_attachment;
		trs right_foot_box_bottom_transform;

		// Head
		{
			head = create_child_entity(ragdoll, "head", head_limb_details, joint_init_ids.head_idx, RagdollLimbType::HEAD, physical_animation_component->neck_chain);

			TransformComponent* transform_component = head.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * head_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = head.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::RAGDOLL;
			capsule_shape_component->material = material;

			head_capsule_bottom_transform = position_capsule_between_joints_from_radius(
				capsule_shape_component,
				settings.head_radius,
				neck_joint_transform,
				head_end_joint_transform,
				neck_joint_transform,
				1.0f,
				settings.head_joint_adjastment);

			create_dynamic_body(head, mass * settings.head_mass_percentage, settings.max_head_contact_impulse);

			if (head_limb_details.motor_drive.has_value())
			{
				head_attachment = create_attachment_dynamic_body(ragdoll, "head_attachment", mass * settings.head_mass_percentage);
				head_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * head_joint_transform);
			}
		}

		// Neck
		{
			neck = create_child_entity(ragdoll, "neck", neck_limb_details, joint_init_ids.neck_idx, RagdollLimbType::NECK, physical_animation_component->neck_chain);

			TransformComponent* transform_component = neck.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * neck_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = neck.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::RAGDOLL;
			capsule_shape_component->material = material;

			neck_capsule_bottom_transform = position_capsule_between_joints_from_radius(
				capsule_shape_component,
				settings.neck_radius,
				neck_joint_transform,
				head_joint_transform,
				neck_joint_transform,
				1.8f,
				settings.neck_joint_adjastment);

			create_dynamic_body(neck, mass * settings.head_mass_percentage, settings.max_head_contact_impulse);

			if (neck_limb_details.motor_drive.has_value())
			{
				neck_attachment = create_attachment_dynamic_body(ragdoll, "neck_attachment", mass * settings.head_mass_percentage);
				neck_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * neck_joint_transform);
			}
		}

		// Body upper (from body middle to neck)
		{
			body_upper = create_child_entity(ragdoll, "body_upper", body_upper_limb_details, joint_init_ids.spine_03_idx, RagdollLimbType::BODY_UPPER, physical_animation_component->body_chain);

			TransformComponent* transform_component = body_upper.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * thorax_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = body_upper.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::RAGDOLL;
			capsule_shape_component->material = material;

			body_upper_capsule_bottom_transform = position_capsule_between_joints_from_height(
				capsule_shape_component,
				distance_between_arms * settings.upper_body_height_modifier,
				middle_abdomen_thorax_transform,
				neck_joint_transform,
				thorax_joint_transform,
				settings.upper_body_radius_modifier,
				settings.thorax_joint_adjastment);

			create_dynamic_body(body_upper, mass * settings.body_upper_mass_percentage, settings.max_body_contact_impulse);

			if (body_upper_limb_details.motor_drive.has_value())
			{
				body_upper_attachment = create_attachment_dynamic_body(ragdoll, "body_upper_attachment", mass * settings.body_upper_mass_percentage);
				body_upper_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * thorax_joint_transform);
			}
		}

		// Body middle
		{
			body_middle = create_child_entity(ragdoll, "body_middle", body_middle_limb_details, joint_init_ids.spine_01_idx, RagdollLimbType::BODY_MIDDLE, physical_animation_component->body_chain);

			TransformComponent* transform_component = body_middle.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * abdomen_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = body_middle.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::RAGDOLL;
			capsule_shape_component->material = material;

			middle_default_transform = position_capsule_between_joints_from_height(
				capsule_shape_component,
				distance_between_arms * settings.middle_body_height_modifier,
				abdomen_joint_transform,
				thorax_joint_transform,
				abdomen_joint_transform,
				settings.middle_body_radius_modifier,
				settings.abdomen_joint_adjastment);

			create_dynamic_body(body_middle, mass * settings.body_lower_mass_percentage, settings.max_body_contact_impulse);
			
			if (body_middle_limb_details.motor_drive.has_value())
			{
				body_middle_attachment = create_attachment_dynamic_body(ragdoll, "body_middle_attachment", mass * settings.body_lower_mass_percentage);
				body_middle_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * abdomen_joint_transform);
			}
		}

		// Body lower (from pelvis to abdomen)
		{
			body_lower = create_child_entity(ragdoll, "pelvis", body_middle_limb_details, joint_init_ids.pelvis_idx, RagdollLimbType::BODY_LOWER, physical_animation_component->body_chain);

			TransformComponent* transform_component = body_lower.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * pelvis_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = body_lower.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::RAGDOLL;
			capsule_shape_component->material = material;

			lower_default_transform = position_capsule_between_joints_from_height(
				capsule_shape_component,
				distance_between_arms * settings.lower_body_height_modifier,
				pelvis_joint_transform,
				middle_abdomen_thorax_transform,
				pelvis_joint_transform,
				settings.lower_body_radius_modifier,
				settings.pelvis_joint_adjastment);

			create_dynamic_body(body_lower, mass * settings.body_lower_mass_percentage, settings.max_body_contact_impulse);
		}

		// Lower body attachment.
		{
			Entity body_lower_ghost = ragdoll.get_world()->create_entity("pelvis_ghost");
			body_lower_ghost.set_parent(ragdoll.get_handle());

			TransformComponent* transform_component = body_lower_ghost.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * pelvis_joint_transform);

			DynamicBodyComponent* lower_body_ghost_component = body_lower_ghost.add_component<DynamicBodyComponent>();
			lower_body_ghost_component->simulated.get_for_write() = false;
			lower_body_ghost_component->kinematic.get_for_write() = true;
			lower_body_ghost_component->kinematic_motion_type.get_for_write() = KinematicMotionType::VELOCITY;
			lower_body_ghost_component->use_gravity.get_for_write() = false;
			lower_body_ghost_component->mass.get_for_write() = body_lower.get_component<DynamicBodyComponent>()->mass;

			SphereShapeComponent* shape_component = body_lower_ghost.add_component<SphereShapeComponent>();
			shape_component->collision_type = CollisionType::NONE;
			shape_component->radius = 0.01f;
			shape_component->use_in_scene_queries = false;

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
				joint_component->stiffness.get_for_write() = 1600.0f;
				joint_component->damping.get_for_write() = 40.0f;
				joint_component->max_distance.get_for_write() = 0.1f;
				joint_component->min_distance.get_for_write() = 0.0f;
			}

			physical_animation_component->attachment_body = EntityPtr{ body_lower_ghost };
		}

		// Left clavicle
		{
			left_clavicle = create_child_entity(ragdoll, "l_clavicle", clavicle_limb_details, joint_init_ids.clavicle_l_idx, RagdollLimbType::CLAVICLE, physical_animation_component->left_arm_chain);

			TransformComponent* transform_component = left_clavicle.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * left_clavicle_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = left_clavicle.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::NONE;
			capsule_shape_component->material = material;

			const trs middle_clavicle_thorax_transform = trs(
				lerp(left_clavicle_joint_transform.position, thorax_joint_transform.position, 0.5f),
				slerp(left_clavicle_joint_transform.rotation, thorax_joint_transform.rotation, 0.5f),
				lerp(left_clavicle_joint_transform.scale, thorax_joint_transform.scale, 0.5f));

			left_clavicle_capsule_bottom_transform = position_capsule_between_joints_from_radius(
				capsule_shape_component,
				settings.arm_radius,
				middle_clavicle_thorax_transform,
				left_arm_joint_transform,
				left_clavicle_joint_transform,
				1.5f);

			create_dynamic_body(left_clavicle, mass * settings.arm_mass_percentage, settings.max_arm_contact_impulse);

			if (clavicle_limb_details.motor_drive.has_value())
			{
				left_clavicle_attachment = create_attachment_dynamic_body(ragdoll, "left_clavicle_attachment", mass * settings.arm_mass_percentage);
				left_clavicle_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * left_clavicle_joint_transform);
			}
		}

		// Left arm
		{
			left_arm = create_child_entity(ragdoll, "l_arm", arm_limb_details, joint_init_ids.upperarm_l_idx, RagdollLimbType::ARM, physical_animation_component->left_arm_chain);

			TransformComponent* transform_component = left_arm.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * left_arm_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = left_arm.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::RAGDOLL;
			capsule_shape_component->material = material;

			left_arm_capsule_bottom_transform = position_capsule_between_joints_from_radius(
				capsule_shape_component,
				settings.arm_radius,
				left_arm_joint_transform,
				left_forearm_joint_transform,
				left_arm_joint_transform,
				1.5f);

			create_dynamic_body(left_arm, mass * settings.arm_mass_percentage, settings.max_arm_contact_impulse);
		
			if (arm_limb_details.motor_drive.has_value())
			{
				left_arm_attachment = create_attachment_dynamic_body(ragdoll, "left_arm_attachment", mass * settings.arm_mass_percentage);
				left_arm_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform* left_arm_joint_transform);
			}
		}

		// Left forearm
		{
			left_forearm = create_child_entity(ragdoll, "l_forearm", forearm_limb_details, joint_init_ids.lowerarm_l_idx, RagdollLimbType::FOREARM, physical_animation_component->left_arm_chain);

			TransformComponent* transform_component = left_forearm.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * left_forearm_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = left_forearm.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::RAGDOLL;
			capsule_shape_component->material = material;

			left_forearm_capsule_bottom_transform = position_capsule_between_joints_from_radius(
				capsule_shape_component,
				settings.forearm_radius,
				left_forearm_joint_transform,
				left_hand_joint_transform,
				left_forearm_joint_transform,
				1.5f);

			create_dynamic_body(left_forearm, mass * settings.forearm_mass_percentage, settings.max_forearm_contact_impulse);
		
			if (forearm_limb_details.motor_drive.has_value())
			{
				left_forearm_attachment = create_attachment_dynamic_body(ragdoll, "left_forearm_attachment", mass * settings.forearm_mass_percentage);
				left_forearm_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * left_forearm_joint_transform);
			}
		}

		// Left hand
		{
			left_hand = create_child_entity(ragdoll, "l_hand", hand_limb_details, joint_init_ids.hand_l_idx, RagdollLimbType::HAND, physical_animation_component->left_arm_chain);

			TransformComponent* transform_component = left_hand.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * left_hand_joint_transform);

			BoxShapeComponent* box_shape_component = left_hand.add_component<BoxShapeComponent>();
			box_shape_component->collision_type = CollisionType::RAGDOLL;
			box_shape_component->material = material;

			left_hand_box_bottom_transform = position_box_between_joints(
				box_shape_component,
				vec3(distance_between_hand_and_hand_end, settings.hand_height, settings.hand_width),
				left_hand_joint_transform,
				left_hand_end_joint_transform,
				left_hand_joint_transform);

			create_dynamic_body(left_hand, mass * settings.hand_mass_percentage, settings.max_hand_contact_impulse);
		
			if (hand_limb_details.motor_drive.has_value())
			{
				left_hand_attachment = create_attachment_dynamic_body(ragdoll, "left_hand_attachment", mass * settings.hand_mass_percentage);
				left_hand_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * left_hand_joint_transform);
			}
		}

		// Right clavicle
		{
			right_clavicle = create_child_entity(ragdoll, "r_clavicle", clavicle_limb_details, joint_init_ids.clavicle_r_idx, RagdollLimbType::CLAVICLE, physical_animation_component->right_arm_chain);

			TransformComponent* transform_component = right_clavicle.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * right_clavicle_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = right_clavicle.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::NONE;
			capsule_shape_component->material = material;

			const trs middle_clavicle_thorax_transform = trs(
				lerp(right_clavicle_joint_transform.position, thorax_joint_transform.position, 0.5f),
				slerp(right_clavicle_joint_transform.rotation, thorax_joint_transform.rotation, 0.5f),
				lerp(right_clavicle_joint_transform.scale, thorax_joint_transform.scale, 0.5f));

			right_clavicle_capsule_bottom_transform = position_capsule_between_joints_from_radius(
				capsule_shape_component,
				settings.arm_radius,
				middle_clavicle_thorax_transform,
				right_arm_joint_transform,
				right_clavicle_joint_transform,
				1.5f);

			create_dynamic_body(right_clavicle, mass * settings.arm_mass_percentage, settings.max_arm_contact_impulse);

			if (clavicle_limb_details.motor_drive.has_value())
			{
				right_clavicle_attachment = create_attachment_dynamic_body(ragdoll, "right_clavicle_attachment", mass * settings.arm_mass_percentage);
				right_clavicle_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * right_clavicle_joint_transform);
			}
		}

		// Right arm
		{
			right_arm = create_child_entity(ragdoll, "r_arm", arm_limb_details, joint_init_ids.upperarm_r_idx, RagdollLimbType::ARM, physical_animation_component->right_arm_chain);

			TransformComponent* transform_component = right_arm.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * right_arm_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = right_arm.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::RAGDOLL;
			capsule_shape_component->material = material;

			right_arm_capsule_bottom_transform = position_capsule_between_joints_from_radius(
				capsule_shape_component,
				settings.arm_radius,
				right_arm_joint_transform,
				right_forearm_joint_transform,
				right_arm_joint_transform,
				1.5f);

			create_dynamic_body(right_arm, mass * settings.arm_mass_percentage, settings.max_arm_contact_impulse);
		
			if (arm_limb_details.motor_drive.has_value())
			{
				right_arm_attachment = create_attachment_dynamic_body(ragdoll, "right_arm_attachment", mass * settings.arm_mass_percentage);
				right_arm_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * right_arm_joint_transform);
			}
		}

		// Right forearm
		{
			right_forearm = create_child_entity(ragdoll, "r_forearm", forearm_limb_details, joint_init_ids.lowerarm_r_idx, RagdollLimbType::FOREARM, physical_animation_component->right_arm_chain);

			TransformComponent* transform_component = right_forearm.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * right_forearm_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = right_forearm.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::RAGDOLL;
			capsule_shape_component->material = material;

			right_forearm_capsule_bottom_transform = position_capsule_between_joints_from_radius(
				capsule_shape_component,
				settings.forearm_radius,
				right_forearm_joint_transform,
				right_hand_joint_transform,
				right_forearm_joint_transform,
				1.5f);

			create_dynamic_body(right_forearm, mass * settings.forearm_mass_percentage, settings.max_forearm_contact_impulse);

			if (forearm_limb_details.motor_drive.has_value())
			{
				right_forearm_attachment = create_attachment_dynamic_body(ragdoll, "right_forearm_attachment", mass * settings.forearm_mass_percentage);
				right_forearm_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * right_forearm_joint_transform);
			}
		}

		// Right hand
		{
			right_hand = create_child_entity(ragdoll, "r_hand", hand_limb_details, joint_init_ids.hand_r_idx, RagdollLimbType::HAND, physical_animation_component->right_arm_chain);

			TransformComponent* transform_component = right_hand.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * right_hand_joint_transform);

			BoxShapeComponent* box_shape_component = right_hand.add_component<BoxShapeComponent>();
			box_shape_component->collision_type = CollisionType::RAGDOLL;
			box_shape_component->material = material;

			right_hand_box_bottom_transform = position_box_between_joints(
				box_shape_component,
				vec3(distance_between_hand_and_hand_end, settings.hand_height, settings.hand_width),
				right_hand_joint_transform,
				right_hand_end_joint_transform,
				right_hand_joint_transform);

			create_dynamic_body(right_hand, mass * settings.hand_mass_percentage, settings.max_hand_contact_impulse);
		
			if (hand_limb_details.motor_drive.has_value())
			{
				right_hand_attachment = create_attachment_dynamic_body(ragdoll, "right_hand_attachment", mass * settings.hand_mass_percentage);
				right_hand_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * right_hand_joint_transform);
			}
		}

		// Left up leg
		{
			left_up_leg = create_child_entity(ragdoll, "l_thigh", leg_limb_details, joint_init_ids.thigh_l_idx, RagdollLimbType::LEG, physical_animation_component->left_leg_chain);

			TransformComponent* transform_component = left_up_leg.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * left_up_leg_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = left_up_leg.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::RAGDOLL;
			capsule_shape_component->material = material;

			left_up_leg_capsule_bottom_transform = position_capsule_between_joints_from_radius(
				capsule_shape_component,
				settings.up_leg_radius * 1.1f,
				left_up_leg_joint_transform,
				left_leg_joint_transform,
				left_up_leg_joint_transform,
				1.1f);

			create_dynamic_body(left_up_leg, mass * settings.up_leg_mass_percentage, settings.max_up_leg_contact_impulse);
		
			if (leg_limb_details.motor_drive.has_value())
			{
				left_up_leg_attachment = create_attachment_dynamic_body(ragdoll, "left_up_leg_attachment", mass * settings.up_leg_mass_percentage);
				left_up_leg_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * left_up_leg_joint_transform);
			}
		}

		// Left leg
		{
			left_leg = create_child_entity(ragdoll, "l_calf", calf_limb_details, joint_init_ids.calf_l_idx, RagdollLimbType::CALF, physical_animation_component->left_leg_chain);

			TransformComponent* transform_component = left_leg.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * left_leg_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = left_leg.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::RAGDOLL;
			capsule_shape_component->material = material;

			left_leg_capsule_bottom_transform = position_capsule_between_joints_from_radius(
				capsule_shape_component,
				settings.leg_radius * 1.2f,
				left_leg_joint_transform,
				left_foot_joint_transform,
				left_leg_joint_transform,
				1.3f);

			create_dynamic_body(left_leg, mass * settings.leg_mass_percentage, settings.max_leg_contact_impulse);
		
			if (calf_limb_details.motor_drive.has_value())
			{
				left_leg_attachment = create_attachment_dynamic_body(ragdoll, "left_leg_attachment", mass * settings.leg_mass_percentage);
				left_leg_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * left_leg_joint_transform);
			}
		}

		// Left foot
		{
			const vec3 foot_end_same_y(left_foot_end_joint_transform.position.x, left_foot_joint_transform.position.y, left_foot_end_joint_transform.position.z);
			const vec3 foot_to_foot_end_offset = foot_end_same_y - left_foot_joint_transform.position;
			const vec3 center = left_foot_joint_transform.position + foot_to_foot_end_offset / 2.0f - vec3(0.0f, distance_between_foot_y_and_foot_end_y / 2.0f, 0.0f);

			left_foot = create_child_entity(ragdoll, "l_foot", foot_limb_details, joint_init_ids.foot_l_idx, RagdollLimbType::FOOT, physical_animation_component->left_leg_chain);

			TransformComponent* transform_component = left_foot.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * left_foot_joint_transform);

			BoxShapeComponent* box_shape_component = left_foot.add_component<BoxShapeComponent>();
			box_shape_component->collision_type = CollisionType::NONE;
			box_shape_component->material = material;

			left_foot_box_bottom_transform = position_box_between_joints(
				box_shape_component,
				vec3(distance_between_foot_and_foot_end, distance_between_foot_y_and_foot_end_y * 2.0f, settings.foot_width),
				trs(center - foot_to_foot_end_offset / 2.0f, left_foot_joint_transform.rotation, left_foot_joint_transform.scale),
				trs(center + foot_to_foot_end_offset / 2.0f, left_foot_joint_transform.rotation, left_foot_joint_transform.scale),
				left_foot_joint_transform);

			create_dynamic_body(left_foot, mass * settings.foot_mass_percentage, settings.max_foot_contact_impulse);
		
			if (foot_limb_details.motor_drive.has_value())
			{
				left_foot_attachment = create_attachment_dynamic_body(ragdoll, "left_foot_attachment", mass * settings.foot_mass_percentage);
				left_foot_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * left_foot_joint_transform);
			}
		}

		// Right up leg
		{
			right_up_leg = create_child_entity(ragdoll, "r_thigh", leg_limb_details, joint_init_ids.thigh_r_idx, RagdollLimbType::LEG, physical_animation_component->right_leg_chain);

			TransformComponent* transform_component = right_up_leg.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * right_up_leg_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = right_up_leg.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::RAGDOLL;
			capsule_shape_component->material = material;

			right_up_leg_capsule_bottom_transform = position_capsule_between_joints_from_radius(
				capsule_shape_component,
				settings.up_leg_radius * 1.1f,
				right_up_leg_joint_transform,
				right_leg_joint_transform,
				right_up_leg_joint_transform,
				1.1f);

			create_dynamic_body(right_up_leg, mass * settings.up_leg_mass_percentage, settings.max_up_leg_contact_impulse);
		
			if (leg_limb_details.motor_drive.has_value())
			{
				right_up_leg_attachment = create_attachment_dynamic_body(ragdoll, "right_up_leg_attachment", mass * settings.up_leg_mass_percentage);
				right_up_leg_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * right_up_leg_joint_transform);
			}
		}

		// Right leg
		{
			right_leg = create_child_entity(ragdoll, "r_calf", calf_limb_details, joint_init_ids.calf_r_idx, RagdollLimbType::CALF, physical_animation_component->right_leg_chain);

			TransformComponent* transform_component = right_leg.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * right_leg_joint_transform);

			CapsuleShapeComponent* capsule_shape_component = right_leg.add_component<CapsuleShapeComponent>();
			capsule_shape_component->collision_type = CollisionType::RAGDOLL;
			capsule_shape_component->material = material;

			right_leg_capsule_bottom_transform = position_capsule_between_joints_from_radius(
				capsule_shape_component,
				settings.leg_radius * 1.2f,
				right_leg_joint_transform,
				right_foot_joint_transform,
				right_leg_joint_transform,
				1.3f);

			create_dynamic_body(right_leg, mass * settings.leg_mass_percentage, settings.max_leg_contact_impulse);
		
			if (calf_limb_details.motor_drive.has_value())
			{
				right_leg_attachment = create_attachment_dynamic_body(ragdoll, "right_leg_attachment", mass * settings.leg_mass_percentage);
				right_leg_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * right_leg_joint_transform);
			}
		}

		// Right foot
		{
			const vec3 foot_end_same_y(right_foot_end_joint_transform.position.x, right_foot_joint_transform.position.y, right_foot_end_joint_transform.position.z);
			const vec3 foot_to_foot_end_offset = foot_end_same_y - right_foot_joint_transform.position;
			const vec3 center = right_foot_joint_transform.position + foot_to_foot_end_offset / 2.0f - vec3(0.0f, distance_between_foot_y_and_foot_end_y / 2.0f, 0.0f);

			right_foot = create_child_entity(ragdoll, "r_foot", foot_limb_details, joint_init_ids.foot_r_idx, RagdollLimbType::FOOT, physical_animation_component->right_leg_chain);

			TransformComponent* transform_component = right_foot.get_component<TransformComponent>();
			transform_component->set_world_transform(ragdoll_world_transform * right_foot_joint_transform);

			BoxShapeComponent* box_shape_component = right_foot.add_component<BoxShapeComponent>();
			box_shape_component->collision_type = CollisionType::NONE;
			box_shape_component->material = material;

			right_foot_box_bottom_transform = position_box_between_joints(
				box_shape_component,
				vec3(distance_between_foot_and_foot_end, distance_between_foot_y_and_foot_end_y * 2.0f, settings.foot_width),
				trs(center - foot_to_foot_end_offset / 2.0f, right_foot_joint_transform.rotation, right_foot_joint_transform.scale),
				trs(center + foot_to_foot_end_offset / 2.0f, right_foot_joint_transform.rotation, right_foot_joint_transform.scale),
				right_foot_joint_transform);

			create_dynamic_body(right_foot, mass * settings.foot_mass_percentage, settings.max_foot_contact_impulse);
		
			if (foot_limb_details.motor_drive.has_value())
			{
				right_foot_attachment = create_attachment_dynamic_body(ragdoll, "right_foot_attachment", mass * settings.foot_mass_percentage);
				right_foot_attachment.get_component<TransformComponent>()->set_world_transform(ragdoll_world_transform * right_foot_joint_transform);
			}
		}

		create_collision_joint(ragdoll, left_arm, body_upper);
		create_collision_joint(ragdoll, right_arm, body_upper);

		// Neck -> head
		create_d6_joint(
			ragdoll,
			neck, head,
			head_capsule_bottom_transform,
			head_capsule_bottom_transform,
			neck_joint_transform,
			head_joint_transform,
			-25.0f, 25.0f,
			20.0f, 20.0f);

		// Body upper -> neck
		create_d6_joint(
			ragdoll,
			body_upper, neck,
			neck_capsule_bottom_transform,
			neck_capsule_bottom_transform,
			thorax_joint_transform,
			neck_joint_transform,
			-15.0f, 15.0f,
			15.0f, 15.0f);

		// Body middle -> body upper
		const float body_upper_forward_angle_deg = 25.0f;
		const float body_upper_backward_angle_deg = 10.0f;
		const float body_upper_d6_swing_y_deg = (body_upper_forward_angle_deg + body_upper_backward_angle_deg) / 2.0f;
		const vec3 body_upper_capsule_y_axis = body_upper_capsule_bottom_transform.rotation * vec3(0.0f, 1.0f, 0.0f);
		const trs body_middle_d6_transform = trs(
			body_upper_capsule_bottom_transform.position,
			quat(body_upper_capsule_y_axis, deg2rad(body_upper_d6_swing_y_deg - body_upper_backward_angle_deg)) * body_upper_capsule_bottom_transform.rotation,
			body_upper_capsule_bottom_transform.scale);
		create_d6_joint(
			ragdoll,
			body_middle,
			body_upper,
			body_middle_d6_transform,
			body_upper_capsule_bottom_transform,
			abdomen_joint_transform,
			thorax_joint_transform,
			-10.0f,
			10.0f,
			body_upper_d6_swing_y_deg,
			15.0f);

		// Body lower -> body middle
		const float body_middle_forward_angle_deg = 8.0f;
		const float body_middle_backward_angle_deg = 4.0f;
		const float body_middle_d6_swing_y_deg = (body_middle_forward_angle_deg + body_middle_backward_angle_deg) / 2.0f;
		const vec3 body_middle_capsule_y_axis = middle_default_transform.rotation * vec3(0.0f, 1.0f, 0.0f);
		const trs body_lower_d6_transform = trs(
			middle_default_transform.position,
			quat(body_middle_capsule_y_axis, deg2rad(body_middle_d6_swing_y_deg - body_middle_backward_angle_deg)) * middle_default_transform.rotation,
			middle_default_transform.scale);
		create_d6_joint(
			ragdoll,
			body_lower,
			body_middle,
			body_lower_d6_transform,
			middle_default_transform,
			pelvis_joint_transform,
			abdomen_joint_transform,
			-4.0f,
			4.0f,
			body_middle_d6_swing_y_deg,
			4.0f);

		create_d6_joint(
			ragdoll,
			body_upper, left_clavicle,
			left_clavicle_joint_transform,
			left_clavicle_joint_transform,
			thorax_joint_transform,
			left_clavicle_joint_transform,
			-160.0f, 160.0f,
			160.0f, 160.0f);

		create_d6_joint(
			ragdoll,
			left_clavicle, left_arm,
			left_arm_capsule_bottom_transform,
			left_arm_capsule_bottom_transform,
			left_clavicle_joint_transform,
			left_arm_joint_transform,
			-160.0f, 160.0f,
			160.0f, 160.0f);

		// Left arm -> left forearm
		const float forearm_d6_swing_y_deg = 55.0f;
		const vec3 left_forearm_capsule_y_axis = left_forearm_capsule_bottom_transform.rotation * vec3(0.0f, 1.0f, 0.0f);
		const trs left_forearm_d6_transform = trs(
			left_forearm_capsule_bottom_transform.position,
			quat(left_forearm_capsule_y_axis, deg2rad(forearm_d6_swing_y_deg)) * left_forearm_capsule_bottom_transform.rotation,
			left_forearm_capsule_bottom_transform.scale);

		create_d6_joint(
			ragdoll,
			left_arm, left_forearm,
			left_forearm_d6_transform,
			left_forearm_capsule_bottom_transform,
			left_arm_joint_transform,
			left_forearm_joint_transform,
			-10.0f, 10.0f,
			forearm_d6_swing_y_deg,
			4.0f);

		// Left forearm -> left hand
		create_d6_joint(
			ragdoll,
			left_forearm, left_hand,
			left_hand_box_bottom_transform,
			left_hand_box_bottom_transform,
			left_forearm_joint_transform,
			left_hand_joint_transform,
			-8.0f, 8.0f,
			22.0f,
			40.0f);

		create_d6_joint(
			ragdoll,
			body_upper, right_clavicle,
			right_clavicle_joint_transform,
			right_clavicle_joint_transform,
			thorax_joint_transform,
			right_clavicle_joint_transform,
			-160.0f, 160.0f,
			160.0f, 160.0f);

		create_d6_joint(
			ragdoll,
			right_clavicle, right_arm,
			right_arm_capsule_bottom_transform,
			right_arm_capsule_bottom_transform,
			right_clavicle_joint_transform,
			right_arm_joint_transform,
			-160.0f, 160.0f,
			160.0f, 160.0f);

		// Right arm -> right forearm
		const vec3 right_forearm_capsule_y_axis = right_forearm_capsule_bottom_transform.rotation * vec3(0.0f, 1.0f, 0.0f);
		const trs right_forearm_d6_transform = trs(
			right_forearm_capsule_bottom_transform.position,
			quat(right_forearm_capsule_y_axis, deg2rad(forearm_d6_swing_y_deg)) * right_forearm_capsule_bottom_transform.rotation,
			right_forearm_capsule_bottom_transform.scale);

		create_d6_joint(
			ragdoll,
			right_arm, right_forearm,
			right_forearm_d6_transform,
			right_forearm_capsule_bottom_transform,
			right_arm_joint_transform,
			right_forearm_joint_transform,
			-10.0f, 10.0f,
			forearm_d6_swing_y_deg,
			4.0f);

		// Right forearm -> right hand
		create_d6_joint(
			ragdoll,
			right_forearm, right_hand,
			right_hand_box_bottom_transform,
			right_hand_box_bottom_transform,
			right_forearm_joint_transform,
			right_hand_joint_transform,
			-8.0f, 8.0f,
			22.0f,
			40.0f);

		// Pelvis -> left up leg
		const float up_leg_back_angle_deg = 5.0; // How far up leg can be rotated around y axis in backwards direction
		const float up_leg_forward_angle_deg = 55.0f; // How far up leg can be rotated around y axis in forward direction
		const float up_leg_d6_swing_y_deg = (up_leg_forward_angle_deg + up_leg_back_angle_deg) / 2.0f;
		const vec3 left_up_leg_capsule_y_axis = left_up_leg_capsule_bottom_transform.rotation * vec3(0.0f, 1.0f, 0.0f);
		const trs left_up_leg_d6_transform = trs(
			left_up_leg_capsule_bottom_transform.position,
			quat(left_up_leg_capsule_y_axis, deg2rad(up_leg_d6_swing_y_deg - up_leg_back_angle_deg)) * left_up_leg_capsule_bottom_transform.rotation,
			left_up_leg_capsule_bottom_transform.scale);
		create_d6_joint(
			ragdoll,
			body_lower, left_up_leg,
			left_up_leg_d6_transform,
			left_up_leg_capsule_bottom_transform,
			pelvis_joint_transform,
			left_up_leg_joint_transform,
			-6.0f, 6.0f,
			up_leg_d6_swing_y_deg,
			35.0f);

		// Left up leg -> left leg
		const float leg_d6_swing_y_deg = 45.0f;
		const vec3 left_leg_capsule_y_axis = left_leg_capsule_bottom_transform.rotation * vec3(0.0f, 1.0f, 0.0f);
		const trs left_leg_d6_transform = trs(
			left_leg_capsule_bottom_transform.position,
			quat(left_leg_capsule_y_axis, deg2rad(leg_d6_swing_y_deg)) * left_leg_capsule_bottom_transform.rotation,
			left_leg_capsule_bottom_transform.scale);
		create_d6_joint(
			ragdoll,
			left_up_leg, left_leg,
			left_leg_d6_transform,
			left_leg_capsule_bottom_transform,
			left_up_leg_joint_transform,
			left_leg_joint_transform,
			-10.0f, 10.0f,
			leg_d6_swing_y_deg,
			5.0f);

		// Left leg -> left foot
		create_d6_joint(
			ragdoll,
			left_leg, left_foot,
			left_foot_box_bottom_transform,
			left_foot_box_bottom_transform,
			left_leg_joint_transform,
			left_foot_joint_transform,
			-17.0f, 17.0f,
			35.0f,
			22.5f);

		// Pelvis -> right up leg
		const vec3 right_up_leg_capsule_y_axis = right_up_leg_capsule_bottom_transform.rotation * vec3(0.0f, 1.0f, 0.0f);
		const trs right_up_leg_d6_transform = trs(
			right_up_leg_capsule_bottom_transform.position,
			quat(right_up_leg_capsule_y_axis, deg2rad(up_leg_d6_swing_y_deg - up_leg_back_angle_deg)) * right_up_leg_capsule_bottom_transform.rotation,
			right_up_leg_capsule_bottom_transform.scale);
		create_d6_joint(
			ragdoll,
			body_lower, right_up_leg,
			right_up_leg_d6_transform,
			right_up_leg_capsule_bottom_transform,
			pelvis_joint_transform,
			right_up_leg_joint_transform,
			-6.0f, 6.0f,
			up_leg_d6_swing_y_deg,
			35.0f);

		// Right up leg -> right leg
		const vec3 right_leg_capsule_y_axis = right_leg_capsule_bottom_transform.rotation * vec3(0.0f, 1.0f, 0.0f);
		const trs right_leg_d6_transform = trs(
			right_leg_capsule_bottom_transform.position,
			quat(right_leg_capsule_y_axis, deg2rad(leg_d6_swing_y_deg)) * right_leg_capsule_bottom_transform.rotation,
			right_leg_capsule_bottom_transform.scale);
		create_d6_joint(
			ragdoll,
			right_up_leg, right_leg,
			right_leg_d6_transform,
			right_leg_capsule_bottom_transform,
			right_up_leg_joint_transform,
			right_leg_joint_transform,
			-10.0f, 10.0f,
			leg_d6_swing_y_deg,
			5.0f);

		// Right leg -> right foot
		create_d6_joint(
			ragdoll,
			right_leg, right_foot,
			right_foot_box_bottom_transform,
			right_foot_box_bottom_transform,
			right_leg_joint_transform,
			right_foot_joint_transform,
			-17.0f, 17.0f,
			35.0f,
			22.5f);

		// Drive joints
		create_drive_joint(ragdoll,
			head_limb_details,
			head_attachment,
			head);

		create_drive_joint(ragdoll,
			neck_limb_details,
			neck_attachment,
			neck);

		create_drive_joint(ragdoll,
			body_upper_limb_details,
			body_upper_attachment,
			body_upper);

		create_drive_joint(ragdoll,
			body_middle_limb_details,
			body_middle_attachment,
			body_middle);

		create_drive_joint(ragdoll,
			clavicle_limb_details,
			left_clavicle_attachment,
			left_clavicle);

		create_drive_joint(ragdoll,
			arm_limb_details,
			left_arm_attachment,
			left_arm);

		create_drive_joint(ragdoll,
			forearm_limb_details,
			left_forearm_attachment,
			left_forearm);

		create_drive_joint(ragdoll,
			hand_limb_details,
			left_hand_attachment,
			left_hand);

		create_drive_joint(ragdoll,
			clavicle_limb_details,
			right_clavicle_attachment,
			right_clavicle);

		create_drive_joint(ragdoll,
			arm_limb_details,
			right_arm_attachment,
			right_arm);

		create_drive_joint(ragdoll,
			forearm_limb_details,
			right_forearm_attachment,
			right_forearm);

		create_drive_joint(ragdoll,
			hand_limb_details,
			right_hand_attachment,
			right_hand);

		create_drive_joint(ragdoll,
			leg_limb_details,
			left_up_leg_attachment,
			left_up_leg);

		create_drive_joint(ragdoll,
			calf_limb_details,
			left_leg_attachment,
			left_leg);

		create_drive_joint(ragdoll,
			foot_limb_details,
			left_foot_attachment,
			left_foot);

		create_drive_joint(ragdoll,
			leg_limb_details,
			right_up_leg_attachment,
			right_up_leg);

		create_drive_joint(ragdoll,
			calf_limb_details,
			right_leg_attachment,
			right_leg);

		create_drive_joint(ragdoll,
			foot_limb_details,
			right_foot_attachment,
			right_foot);
	}

}