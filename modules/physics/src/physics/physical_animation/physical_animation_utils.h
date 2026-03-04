#pragma once

#include "physics_api.h"
#include "physics/physical_animation/physical_animation_component.h"

#include <core/math.h>

namespace era_engine::animation
{
	class SkeletonComponent;
}

namespace era_engine::physics
{
	class ERA_PHYSICS_API PhysicalAnimationUtils final
	{
	public:
		PhysicalAnimationUtils() = delete;

		static vec3 calculate_delta_rotation_time_derivative(const quat& delta_rotation,
			float dt);

		static bool is_body_limb(RagdollLimbType type);
		static bool is_head_limb(RagdollLimbType type);
		static bool is_leg_limb(RagdollLimbType type);
		static bool is_arm_limb(RagdollLimbType type);

		static void reset_motor_drive(PhysicalAnimationLimbComponent* limb_component);

		static void set_simulation_for_limb(const PhysicalAnimationLimbComponent* limb_component,
			bool enable_simulation,
			bool enable_gravity = false);

		static void force_sync_limb_to_skeleton(PhysicalAnimationLimbComponent* limb_component,
			const animation::SkeletonComponent* skeleton_component,
			const trs& ragdoll_world_transform);
	};
}