#pragma once

#include "physics_api.h"
#include "physics/ragdolls/ragdoll_component.h"

#include <animation/animation.h>

#include <ecs/reflection.h>

namespace era_engine::physics
{
	enum class DismemberableLimbType : uint8
	{
		ROOT,
		DISMEMBERABLE,
	};

	struct ERA_PHYSICS_API DismemberableLimbDetails
	{
		float max_health = 0.0f;

		RagdollLimbType limb_type = RagdollLimbType::BODY_LOWER;

		DismemberableLimbType type = DismemberableLimbType::DISMEMBERABLE;
	};

	class ERA_PHYSICS_API RagdollDismembermentProfile
	{
	public:
		RagdollDismembermentProfile() = default;

		const DismemberableLimbDetails* get_details_by_limb_type(RagdollLimbType limb_type) const;

		std::vector<DismemberableLimbDetails> limbs_details;

		ERA_REFLECT
	};
}