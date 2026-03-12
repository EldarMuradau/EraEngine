#include "physics/physical_animation/physical_ragdoll_builder.h"

#include "physics/core/physics.h"
#include "physics/body_component.h"
#include "physics/shape_component.h"
#include "physics/core/physics_utils.h"
#include "physics/shape_utils.h"
#include "physics/physical_animation/physical_animation_component.h"
#include "physics/aggregate_holder_component.h"
#include "physics/joint.h"

#include <core/math.h>
#include <core/debug/debug_var.h>

#include <ecs/base_components/transform_component.h>
#include <ecs/world.h>

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
		child.set_parent(parent.get_handle());

		if (is_physically_animated)
		{
			PhysicalAnimationLimbComponent* limb_component = child.add_component<PhysicalAnimationLimbComponent>();
			limb_component->joint_id = joint_id;
			limb_component->type = type;

			if (details->motor_drive.has_value())
			{
				const MotorDriveDetails& motor_drive = details->motor_drive.value();

				limb_component->angular_range = motor_drive.angular_range;
				limb_component->linear_range = motor_drive.linear_range;
				limb_component->angular_damping_range = motor_drive.angular_damping_range;
				limb_component->linear_damping_range = motor_drive.linear_damping_range;
			}

			limb_component->ragdoll_ptr = EntityPtr{ parent };
		}
		else
		{
			RagdollLimbComponent* limb_component = child.add_component<RagdollLimbComponent>();
			limb_component->joint_id = joint_id;
			limb_component->type = type;
		}

		ragdoll_component->limbs.push_back(EntityPtr{ child });

		if (chain != nullptr)
		{
			chain->connected_limbs.push_back(EntityPtr{ child });
		}

		return child;
	}

	static DynamicBodyComponent* create_dynamic_body(
		bool is_physically_animated,
		Entity& entity,
		const float mass,
		const float max_contact_impulse = 400)
	{
		DynamicBodyComponent* dynamic_body_component = entity.add_component<DynamicBodyComponent>();
		dynamic_body_component->mass.get_for_write() = mass;
		dynamic_body_component->ccd.get_for_write() = true;
		dynamic_body_component->max_depenetration_velocity = 100.0f;
		dynamic_body_component->use_gravity.get_for_write() = !is_physically_animated;
		dynamic_body_component->simulated.get_for_write() = false;
		dynamic_body_component->linear_damping.get_for_write() = 0.15f;
		dynamic_body_component->angular_damping.get_for_write() = 0.25f;
		dynamic_body_component->max_contact_impulse.get_for_write() = max_contact_impulse;
		dynamic_body_component->solver_position_iterations_count.get_for_write() = 16;
		dynamic_body_component->solver_velocity_iterations_count.get_for_write() = 4;
		dynamic_body_component->sleep_threshold.get_for_write() = 0.01f;

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
		float twist_min_deg = -180.0f,
		float twist_max_deg = 180.0f,
		float swing_y_deg = 180.0f,
		float swing_z_deg = 180.0f)
	{
		trs e0_to_e0_joint_transform = invert(e0_joint.joint_object_space_transform) * e0_joint_transform;
		e0_to_e0_joint_transform.rotation = normalize(e0_to_e0_joint_transform.rotation);
		e0_to_e0_joint_transform.scale = vec3(1.0f);

		trs e1_to_e1_joint_transform = invert(e1_joint.joint_object_space_transform) * e1_joint_transform;
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
		joint_entity.set_parent(source.get_handle());

		D6JointComponent* joint_component = joint_entity.add_component<D6JointComponent>(descriptor);

		joint_component->enable_collision.get_for_write() = false;

		joint_component->linear_x_motion_type.get_for_write() = D6JointComponent::Motion::LOCKED;
		joint_component->linear_y_motion_type.get_for_write() = D6JointComponent::Motion::LOCKED;
		joint_component->linear_z_motion_type.get_for_write() = D6JointComponent::Motion::LOCKED;

		if (is_physiclly_animated)
		{
			e0.get_component<PhysicalAnimationLimbComponent>()->joint_entity_ptr = EntityPtr{ joint_entity };
			e1.get_component<PhysicalAnimationLimbComponent>()->parent_joint_component = ComponentPtr{ joint_component };
		}
		else
		{
			e0.get_component<RagdollLimbComponent>()->joint_entity_ptr = EntityPtr{ joint_entity };
		}

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

			joint_component->twist_limit_damping.get_for_write() = 50.0f;
			joint_component->twist_limit_stiffness.get_for_write() = 500.0f;
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
			joint_component->swing_limit_damping.get_for_write() = 50.0f;
			joint_component->swing_limit_stiffness.get_for_write() = 500.0f;
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

		PhysicalAnimationLimbComponent* e1_limb_component = e1.get_component<PhysicalAnimationLimbComponent>();

		e1_limb_component->drive_joint_entity_ptr = EntityPtr{ joint_entity };

		D6JointComponent* joint_component = joint_entity.add_component<D6JointComponent>(descriptor);

		joint_component->perform_slerp_drive = motor_drive.enable_slerp_drive;

		joint_component->enable_collision.get_for_write() = false;
		joint_component->drive_limits_are_forces.get_for_write() = true;

		joint_component->linear_x_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
		joint_component->linear_y_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
		joint_component->linear_z_motion_type.get_for_write() = D6JointComponent::Motion::FREE;

		joint_component->twist_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
		joint_component->swing_y_motion_type.get_for_write() = D6JointComponent::Motion::FREE;
		joint_component->swing_z_motion_type.get_for_write() = D6JointComponent::Motion::FREE;

		joint_component->linear_drive_stiffness = motor_drive.linear_drive_stiffness * RagdollStrengthConfig::LINEAR_STIFFNESS_MODIFIER;
		joint_component->linear_drive_damping = motor_drive.linear_damping_range.x * RagdollStrengthConfig::LINEAR_DAMPING_MODIFIER;
		joint_component->linear_drive_force_limit = motor_drive.max_force;
		joint_component->linear_drive_accelerated = motor_drive.accelerated;

		e1_limb_component->drive_joint_component = ComponentPtr{ joint_component };

		if (motor_drive.enable_slerp_drive)
		{
			joint_component->slerp_drive_force_limit.get_for_write() = motor_drive.max_force;
			joint_component->slerp_drive_stiffness.get_for_write() = motor_drive.angular_drive_stiffness * RagdollStrengthConfig::ANGULAR_STIFFNESS_MODIFIER;
			joint_component->slerp_drive_damping.get_for_write() = motor_drive.angular_damping_range.x * RagdollStrengthConfig::ANGULAR_DAMPING_MODIFIER;
			joint_component->slerp_drive_accelerated.get_for_write() = motor_drive.accelerated;
		}
		else
		{
			joint_component->swing_drive_force_limit.get_for_write() = motor_drive.max_force;
			joint_component->swing_drive_stiffness.get_for_write() = motor_drive.angular_drive_stiffness * RagdollStrengthConfig::ANGULAR_STIFFNESS_MODIFIER;
			joint_component->swing_drive_damping.get_for_write() = motor_drive.angular_damping_range.x * RagdollStrengthConfig::ANGULAR_DAMPING_MODIFIER;
			joint_component->swing_drive_accelerated.get_for_write() = motor_drive.accelerated;

			joint_component->twist_drive_stiffness.get_for_write() = motor_drive.angular_drive_stiffness * RagdollStrengthConfig::ANGULAR_STIFFNESS_MODIFIER;
			joint_component->twist_drive_damping.get_for_write() = motor_drive.angular_damping_range.x * RagdollStrengthConfig::ANGULAR_DAMPING_MODIFIER;
			joint_component->twist_drive_force_limit.get_for_write() = motor_drive.max_force;
			joint_component->twist_drive_accelerated.get_for_write() = motor_drive.accelerated;
		}
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
			shortest_arc(vec3(1.0f, 0.0f, 0.0f), direction) * local_rotation_spin,
			vec3(1.0f));
		const trs box_transform_relative_to_owner = invert(owner_joint_transform) * box_transform;

		box_shape_component->local_position.get_for_write() = box_transform_relative_to_owner.position;
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
		const vec3& local_position_adjustment = vec3::zero,
		const quat& local_rotation_spin = quat::identity)
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
			center_between_joints + local_position_adjustment,
			shortest_arc(vec3(1.0f, 0.0f, 0.0f), direction) * local_rotation_spin,
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
			shortest_arc(vec3(1.0f, 0.0f, 0.0f), direction) * local_rotation_spin,
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
			center_between_joints  + local_position_adjustment,
			shortest_arc(vec3(0.0f, 1.0f, 0.0f), direction) * local_rotation_spin,
			vec3(1.0f));
		const trs capsule_transform_relative_to_owner = invert(owner_joint_transform) * capsule_transform;

		capsule_shape_component->local_position.get_for_write() = capsule_transform_relative_to_owner.position;
		capsule_shape_component->local_rotation.get_for_write() = capsule_transform_relative_to_owner.rotation;

		const vec3 capsule_x_axis = capsule_transform.rotation * vec3(1.0f, 0.0f, 0.0f);
		return trs(capsule_transform.position - direction * radius, capsule_transform.rotation, capsule_transform.scale) * trs(vec3::zero, shortest_arc(capsule_x_axis, direction), vec3(1.0f));
	}

	void PhysicalRagdollBuilder::build_ragdoll(const RuntimeContext& ctx)
	{
		if (ctx.ragdoll_component->mass <= 0.0f)
		{
			ASSERT(false);
			return;
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

		const ref<Skeleton>& skeleton = ctx.ragdoll.get_component<SkeletonComponent>()->skeleton;
		if (skeleton == nullptr || 
			skeleton->load_state != AssetLoadState::LOADED)
		{
			ASSERT(false);
			return false;
		}

		auto prepare_joint = [&ctx, &skeleton](RagdollSkeletonStructure::JointStructure& joint, uint32 joint_id)
			{
				joint.joint_id = joint_id;
				joint.joint_object_space_transform = SkeletonUtils::get_object_space_joint_transform(skeleton.get(), joint.joint_id);
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

		AggregateHolderComponent* aggregate_component = ctx.ragdoll.add_component<AggregateHolderComponent>();
		aggregate_component->enable_self_collision = true;
		aggregate_component->max_actors = 48;

		ref<PhysicsMaterial> material = PhysicsHolder::physics_ref->create_material(0.3f, 0.5f, 0.7f);
		ASSERT(material != nullptr);

		const RagdollSettings& settings = ctx.ragdoll_component->settings;
		RagdollSkeletonStructure& structure = ctx.skeleton_structure;

		const SkeletonComponent* skeleton_component = ctx.ragdoll.get_component<SkeletonComponent>();

		auto create_capsule_limb_between_joints = [&ctx, &material, &skeleton_component](
			uint32 joint_id,
			const PhysicalLimbDetails* details,
			float mass_percentage,
			float max_contact_impulse,
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

				CapsuleShapeComponent* capsule_shape_component = create_shape<CapsuleShapeComponent>(limb, material, collision_type);

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
					max_contact_impulse);
			};

		auto create_box_limb_between_joints = [&ctx, &material, &skeleton_component](
			uint32 joint_id,
			const PhysicalLimbDetails* details,
			float mass_percentage,
			float max_contact_impulse,
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

				BoxShapeComponent* box_shape_component = create_shape<BoxShapeComponent>(limb, material, collision_type);

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
					max_contact_impulse);
			};

		auto create_capsule_limb_between_joints_from_radius = [&ctx, &material, &skeleton_component](
			uint32 joint_id,
			const PhysicalLimbDetails* details,
			float mass_percentage,
			float max_contact_impulse,
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

				CapsuleShapeComponent* capsule_shape_component = create_shape<CapsuleShapeComponent>(limb, material, collision_type);

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
					max_contact_impulse);
			};

		PhysicalAnimationComponent* physical_animation_component = ctx.enable_physical_animation ? dynamic_cast<PhysicalAnimationComponent*>(ctx.ragdoll_component) : nullptr;

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.head_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->head_limb_details : nullptr,
			settings.mass_settings.head_mass_percentage,
			settings.impulse_settings.max_head_contact_impulse,
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
			settings.local_shape_settings.head_joint_spin
		);

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.neck_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->neck_limb_details : nullptr,
			settings.mass_settings.neck_mass_percentage,
			settings.impulse_settings.max_neck_contact_impulse,
			RagdollLimbType::NECK,
			structure.neck_joint,
			structure.head_joint,
			settings.shapes_settings.neck_radius,
			1.0f,
			true,
			ctx.enable_physical_animation ? physical_animation_component->neck_chain.get() : nullptr,
			CollisionType::RAGDOLL,
			settings.object_space_settings.neck_joint_adjastment,
			settings.object_space_settings.head_joint_adjastment,
			settings.local_shape_settings.neck_joint_adjastment,
			settings.local_shape_settings.neck_joint_spin
		);

		create_capsule_limb_between_joints_from_radius(
			ctx.ragdoll_component->joint_init_ids.spine_03_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->body_upper_limb_details : nullptr,
			settings.mass_settings.body_upper_mass_percentage,
			settings.impulse_settings.max_body_contact_impulse,
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
			settings.local_shape_settings.thorax_joint_spin
		);

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.spine_01_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->body_middle_limb_details : nullptr,
			settings.mass_settings.body_middle_mass_percentage,
			settings.impulse_settings.max_body_contact_impulse,
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
			settings.local_shape_settings.abdomen_joint_spin
		);

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.pelvis_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->body_lower_limb_details : nullptr,
			settings.mass_settings.body_lower_mass_percentage,
			settings.impulse_settings.max_body_contact_impulse,
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
			settings.local_shape_settings.pelvis_joint_spin
		);

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
				joint_component->stiffness.get_for_write() = 800.0f;
				joint_component->damping.get_for_write() = 80.0f;
				joint_component->max_distance.get_for_write() = 0.1f;
				joint_component->min_distance.get_for_write() = 0.0f;
			}

			physical_animation_component->attachment_body = EntityPtr{ body_lower_ghost };
		}

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.clavicle_l_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->clavicle_limb_details : nullptr,
			settings.mass_settings.clavicle_mass_percentage,
			settings.impulse_settings.max_arm_contact_impulse,
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
			settings.local_shape_settings.left_clavicle_joint_spin
		);

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.upperarm_l_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->arm_limb_details : nullptr,
			settings.mass_settings.arm_mass_percentage,
			settings.impulse_settings.max_arm_contact_impulse,
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
			settings.local_shape_settings.left_arm_joint_spin
		);

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.lowerarm_l_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->forearm_limb_details : nullptr,
			settings.mass_settings.forearm_mass_percentage,
			settings.impulse_settings.max_forearm_contact_impulse,
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
			settings.local_shape_settings.left_hand_joint_spin
		);

		create_box_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.hand_l_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->hand_limb_details : nullptr,
			settings.mass_settings.hand_mass_percentage,
			settings.impulse_settings.max_hand_contact_impulse,
			RagdollLimbType::HAND,
			structure.left_hand_joint,
			structure.left_hand_end_joint,
			vec3(settings.shapes_settings.hand_x, settings.shapes_settings.hand_y, settings.shapes_settings.hand_z),
			ctx.enable_physical_animation ? physical_animation_component->left_arm_chain.get() : nullptr,
			CollisionType::RAGDOLL,
			settings.object_space_settings.left_hand_joint_adjastment,
			vec3::zero,
			settings.local_shape_settings.left_hand_joint_adjastment,
			settings.local_shape_settings.left_hand_joint_spin
		);

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.clavicle_r_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->clavicle_limb_details : nullptr,
			settings.mass_settings.clavicle_mass_percentage,
			settings.impulse_settings.max_arm_contact_impulse,
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
			settings.local_shape_settings.right_clavicle_joint_spin
		);

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.upperarm_r_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->arm_limb_details : nullptr,
			settings.mass_settings.arm_mass_percentage,
			settings.impulse_settings.max_arm_contact_impulse,
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
			settings.local_shape_settings.right_arm_joint_spin
		);

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.lowerarm_r_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->forearm_limb_details : nullptr,
			settings.mass_settings.forearm_mass_percentage,
			settings.impulse_settings.max_forearm_contact_impulse,
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
			settings.local_shape_settings.right_hand_joint_spin
		);

		create_box_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.hand_r_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->hand_limb_details : nullptr,
			settings.mass_settings.hand_mass_percentage,
			settings.impulse_settings.max_hand_contact_impulse,
			RagdollLimbType::HAND,
			structure.right_hand_joint,
			structure.right_hand_end_joint,
			vec3(settings.shapes_settings.hand_x, settings.shapes_settings.hand_y, settings.shapes_settings.hand_z),
			ctx.enable_physical_animation ? physical_animation_component->right_arm_chain.get() : nullptr,
			CollisionType::RAGDOLL,
			settings.object_space_settings.right_hand_joint_adjastment,
			vec3::zero,
			settings.local_shape_settings.right_hand_joint_adjastment,
			settings.local_shape_settings.right_hand_joint_spin
		);

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.thigh_l_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->leg_limb_details : nullptr,
			settings.mass_settings.leg_mass_percentage,
			settings.impulse_settings.max_leg_contact_impulse,
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
			settings.local_shape_settings.left_thigh_joint_spin
		);

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.calf_l_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->calf_limb_details : nullptr,
			settings.mass_settings.calf_mass_percentage,
			settings.impulse_settings.max_calf_contact_impulse,
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
			settings.local_shape_settings.left_calf_joint_spin
		);

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.foot_l_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->foot_limb_details : nullptr,
			settings.mass_settings.foot_mass_percentage,
			settings.impulse_settings.max_foot_contact_impulse,
			RagdollLimbType::FOOT,
			structure.left_foot_joint,
			structure.left_foot_end_joint,
			settings.shapes_settings.foot_radius,
			1.0f,
			true,
			ctx.enable_physical_animation ? physical_animation_component->left_leg_chain.get() : nullptr,
			CollisionType::RAGDOLL,
			settings.object_space_settings.left_foot_joint_adjastment,
			settings.object_space_settings.left_foot_end_joint_adjastment,
			settings.local_shape_settings.left_foot_joint_adjastment,
			settings.local_shape_settings.left_foot_joint_spin
		);

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.thigh_r_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->leg_limb_details : nullptr,
			settings.mass_settings.leg_mass_percentage,
			settings.impulse_settings.max_leg_contact_impulse,
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
			settings.local_shape_settings.right_thigh_joint_spin
		);

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.calf_r_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->calf_limb_details : nullptr,
			settings.mass_settings.calf_mass_percentage,
			settings.impulse_settings.max_calf_contact_impulse,
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
			settings.local_shape_settings.right_calf_joint_spin
		);

		create_capsule_limb_between_joints(
			ctx.ragdoll_component->joint_init_ids.foot_r_idx,
			ctx.enable_physical_animation ? &ctx.default_profile->foot_limb_details : nullptr,
			settings.mass_settings.foot_mass_percentage,
			settings.impulse_settings.max_foot_contact_impulse,
			RagdollLimbType::FOOT,
			structure.right_foot_joint,
			structure.right_foot_end_joint,
			settings.shapes_settings.foot_radius,
			1.0f,
			true,
			ctx.enable_physical_animation ? physical_animation_component->right_leg_chain.get() : nullptr,
			CollisionType::RAGDOLL,
			settings.object_space_settings.right_foot_joint_adjastment,
			settings.object_space_settings.right_foot_end_joint_adjastment,
			settings.local_shape_settings.right_foot_joint_adjastment,
			settings.local_shape_settings.right_foot_joint_spin
		);

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

		const trs& body_upper_capsule_bottom_transform = structure.thorax_joint.constraint_object_space_transform.value();
		const trs& middle_default_transform = structure.abdomen_joint.constraint_object_space_transform.value();

		const trs& left_clavicle_capsule_bottom_transform = structure.left_clavicle_joint.constraint_object_space_transform.value();
		const trs& left_arm_capsule_bottom_transform = structure.left_arm_joint.constraint_object_space_transform.value();
		const trs& left_forearm_capsule_bottom_transform = structure.left_forearm_joint.constraint_object_space_transform.value();
		const trs& left_hand_capsule_bottom_transform = structure.left_hand_joint.constraint_object_space_transform.value();

		const trs& right_clavicle_capsule_bottom_transform = structure.right_clavicle_joint.constraint_object_space_transform.value();
		const trs& right_arm_capsule_bottom_transform = structure.right_arm_joint.constraint_object_space_transform.value();
		const trs& right_forearm_capsule_bottom_transform = structure.right_forearm_joint.constraint_object_space_transform.value();
		const trs& right_hand_capsule_bottom_transform = structure.right_hand_joint.constraint_object_space_transform.value();

		const trs& left_up_leg_capsule_bottom_transform = structure.left_leg_joint.constraint_object_space_transform.value();
		const trs& left_leg_capsule_bottom_transform = structure.left_calf_joint.constraint_object_space_transform.value();
		const trs& left_foot_capsule_bottom_transform = structure.left_foot_joint.constraint_object_space_transform.value();

		const trs& right_up_leg_capsule_bottom_transform = structure.right_leg_joint.constraint_object_space_transform.value();
		const trs& right_leg_capsule_bottom_transform = structure.right_calf_joint.constraint_object_space_transform.value();
		const trs& right_foot_capsule_bottom_transform = structure.right_foot_joint.constraint_object_space_transform.value();

		// Neck -> head
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.neck_joint, 
			structure.head_joint,
			head_capsule_bottom_transform,
			head_capsule_bottom_transform,
			-25.0f, 25.0f,
			20.0f, 20.0f);

		// Body upper -> neck
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.thorax_joint, 
			structure.neck_joint,
			neck_capsule_bottom_transform,
			neck_capsule_bottom_transform,
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
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.abdomen_joint,
			structure.thorax_joint,
			body_middle_d6_transform,
			body_upper_capsule_bottom_transform,
			-10.0f,
			10.0f,
			body_upper_d6_swing_y_deg, 15.0f);

		// Body lower -> body middle
		const float body_middle_forward_angle_deg = 12.0f;
		const float body_middle_backward_angle_deg = 8.0f;
		const float body_middle_d6_swing_y_deg = (body_middle_forward_angle_deg + body_middle_backward_angle_deg) / 2.0f;
		const vec3 body_middle_capsule_y_axis = middle_default_transform.rotation * vec3(0.0f, 1.0f, 0.0f);
		const trs body_lower_d6_transform = trs(
			middle_default_transform.position,
			quat(body_middle_capsule_y_axis, deg2rad(body_middle_d6_swing_y_deg - body_middle_backward_angle_deg)) * middle_default_transform.rotation,
			middle_default_transform.scale);
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.pelvis_joint,
			structure.abdomen_joint,
			body_lower_d6_transform,
			middle_default_transform,
			-10.0f,
			10.0f,
			body_middle_d6_swing_y_deg, 10.0f);

		// Body upper -> left clavicle
		const float clavicle_d6_swing_y_deg = 20.0f;
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.thorax_joint,
			structure.left_clavicle_joint,
			left_clavicle_capsule_bottom_transform,
			left_clavicle_capsule_bottom_transform,
			-25.0f, 25.0f,
			clavicle_d6_swing_y_deg, 20.0f);

		// Left clavicle -> left arm
		float arm_forward_angle_deg = 90.0f;  // How far an arm can be rotated forward around Y axis
		const float arm_backward_angle_deg = 22.5f; // How far an arm can be rotated backwards around Y axis
		const float arm_d6_swing_y_deg = (arm_forward_angle_deg + arm_backward_angle_deg) / 2.0f;
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.left_clavicle_joint,
			structure.left_arm_joint,
			left_arm_capsule_bottom_transform,
			left_arm_capsule_bottom_transform,
			-55.0f, 55.0f,
			arm_d6_swing_y_deg, 60.0f);

		// Left arm -> left forearm
		const float forearm_d6_swing_y_deg = 55.0f;
		const vec3 left_forearm_capsule_y_axis = left_forearm_capsule_bottom_transform.rotation * vec3(0.0f, 1.0f, 0.0f);
		const trs left_forearm_d6_transform = trs(
			left_forearm_capsule_bottom_transform.position,
			quat(left_forearm_capsule_y_axis, deg2rad(35.0f)) * left_forearm_capsule_bottom_transform.rotation,
			left_forearm_capsule_bottom_transform.scale);
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.left_arm_joint,
			structure.left_forearm_joint,
			left_forearm_d6_transform,
			left_forearm_capsule_bottom_transform,
			-25.0f, 25.0f,
			forearm_d6_swing_y_deg, 20.0f);

		// Left forearm -> left hand
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.left_forearm_joint,
			structure.left_hand_joint,
			structure.left_hand_joint.joint_object_space_transform,
			left_hand_capsule_bottom_transform,
			-180.0f, 180.0f,
			180.0f, 180.0f);

		// Body upper -> right clavicle
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.thorax_joint,
			structure.right_clavicle_joint,
			right_clavicle_capsule_bottom_transform,
			right_clavicle_capsule_bottom_transform,
			-25.0f, 25.0f,
			clavicle_d6_swing_y_deg, 20.0f);

		// Right clavicle -> right arm
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.right_clavicle_joint,
			structure.right_arm_joint,
			right_arm_capsule_bottom_transform,
			right_arm_capsule_bottom_transform,
			-55.0f, 55.0f,
			arm_d6_swing_y_deg, 60.0f);

		// Right arm -> right forearm
		const vec3 right_forearm_capsule_y_axis = right_forearm_capsule_bottom_transform.rotation * vec3(0.0f, 1.0f, 0.0f);
		const trs right_forearm_d6_transform = trs(
			right_forearm_capsule_bottom_transform.position,
			quat(right_forearm_capsule_y_axis, deg2rad(35.0f)) * right_forearm_capsule_bottom_transform.rotation,
			right_forearm_capsule_bottom_transform.scale);
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.right_arm_joint,
			structure.right_forearm_joint,
			right_forearm_d6_transform,
			right_forearm_capsule_bottom_transform,
			-25.0f, 25.0f,
			forearm_d6_swing_y_deg, 20.0f);

		// Right forearm -> right hand
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.right_forearm_joint,
			structure.right_hand_joint,
			structure.right_hand_joint.joint_object_space_transform,
			right_hand_capsule_bottom_transform,
			-180.0f, 180.0f,
			180.0f, 180.0f);

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
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.pelvis_joint,
			structure.left_leg_joint,
			left_up_leg_d6_transform,
			left_up_leg_capsule_bottom_transform,
			-6.0f, 6.0f,
			up_leg_d6_swing_y_deg, 35.0f);

		// Left up leg -> left leg
		const float leg_d6_swing_y_deg = 45.0f;
		const vec3 left_leg_capsule_y_axis = left_leg_capsule_bottom_transform.rotation * vec3(0.0f, 1.0f, 0.0f);
		const trs left_leg_d6_transform = trs(
			left_leg_capsule_bottom_transform.position,
			quat(left_leg_capsule_y_axis, deg2rad(leg_d6_swing_y_deg)) * left_leg_capsule_bottom_transform.rotation,
			left_leg_capsule_bottom_transform.scale);
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.left_leg_joint,
			structure.left_calf_joint,
			left_leg_d6_transform,
			left_leg_capsule_bottom_transform,
			-10.0f, 10.0f,
			leg_d6_swing_y_deg, 5.0f);

		// Left leg -> left foot
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.left_calf_joint, 
			structure.left_foot_joint,
			left_foot_capsule_bottom_transform,
			left_foot_capsule_bottom_transform,
			-17.0f, 17.0f,
			35.0f, 22.5f);

		// Pelvis -> right up leg
		const vec3 right_up_leg_capsule_y_axis = right_up_leg_capsule_bottom_transform.rotation * vec3(0.0f, 1.0f, 0.0f);
		const trs right_up_leg_d6_transform = trs(
			right_up_leg_capsule_bottom_transform.position,
			quat(right_up_leg_capsule_y_axis, deg2rad(up_leg_d6_swing_y_deg - up_leg_back_angle_deg)) * right_up_leg_capsule_bottom_transform.rotation,
			right_up_leg_capsule_bottom_transform.scale);
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.pelvis_joint,
			structure.right_leg_joint,
			right_up_leg_d6_transform,
			right_up_leg_capsule_bottom_transform,
			-6.0f, 6.0f,
			up_leg_d6_swing_y_deg, 35.0f);

		// Right up leg -> right leg
		const vec3 right_leg_capsule_y_axis = right_leg_capsule_bottom_transform.rotation * vec3(0.0f, 1.0f, 0.0f);
		const trs right_leg_d6_transform = trs(
			right_leg_capsule_bottom_transform.position,
			quat(right_leg_capsule_y_axis, deg2rad(leg_d6_swing_y_deg)) * right_leg_capsule_bottom_transform.rotation,
			right_leg_capsule_bottom_transform.scale);
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.right_leg_joint,
			structure.right_calf_joint,
			right_leg_d6_transform,
			right_leg_capsule_bottom_transform,
			-10.0f, 10.0f,
			leg_d6_swing_y_deg, 5.0f);

		// Right leg -> right foot
		create_d6_joint(
			ctx.enable_physical_animation,
			ctx.ragdoll,
			structure.right_calf_joint,
			structure.right_foot_joint,
			right_foot_capsule_bottom_transform,
			right_foot_capsule_bottom_transform,
			-17.0f, 17.0f,
			35.0f, 22.5f);

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
}