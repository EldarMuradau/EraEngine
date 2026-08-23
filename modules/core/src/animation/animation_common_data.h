#pragma once

#include "core_api.h"

#include "core/math.h"
#include "core/serialization/binary_serializer.h"

#define INVALID_JOINT 0xFFFFFFFF

namespace era_engine::animation
{
	enum LimbType
	{
		limb_type_none,

		limb_type_torso,
		limb_type_head,

		limb_type_upper_arm_right,
		limb_type_lower_arm_right,
		limb_type_hand_right,

		limb_type_upper_arm_left,
		limb_type_lower_arm_left,
		limb_type_hand_left,

		limb_type_upper_leg_right,
		limb_type_lower_leg_right,
		limb_type_foot_right,

		limb_type_upper_leg_left,
		limb_type_lower_leg_left,
		limb_type_foot_left,

		limb_type_count,
	};

	struct ERA_CORE_API SkeletonJoint final
	{
		std::string name;
		LimbType limb_type;
		bool ik = false;

		mat4 inv_bind_transform; // Transforms from model space to joint space.
		mat4 bind_transform; // Position of joint relative to model space.

		uint32 parent_id = INVALID_JOINT;

		ERA_BINARY_SERIALIZE(name, limb_type, ik, inv_bind_transform, bind_transform, parent_id)
	};

	struct ERA_CORE_API SkinningWeights
	{
		SkinningWeights() = default;

		std::array<uint8, 4> skin_indices;
		std::array<uint8, 4> skin_weights;

		ERA_BINARY_SERIALIZE(skin_indices, skin_weights)
	};

	struct ERA_CORE_API SkeletonAssetImportData
	{
		std::vector<animation::SkeletonJoint> joints;
		std::unordered_map<std::string, uint32> name_to_joint_id;
	};
}