#include "physics/physical_animation/dismemberment/ragdoll_dismemberment_profile.h"

#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<RagdollDismembermentProfile>("RagdollDismembermentProfile")
			.constructor<>();
	}

	const DismemberableLimbDetails* RagdollDismembermentProfile::get_details_by_limb_type(RagdollLimbType limb_type) const
	{
		auto iter = std::find_if(limbs_details.begin(), limbs_details.end(), 
			[limb_type] (const DismemberableLimbDetails& details) -> bool
			{
				return static_cast<uint8>(details.limb_type) == static_cast<uint8>(limb_type);
			});

		if (iter == limbs_details.end())
		{
			return nullptr;
		}

		return &*iter;
	}

}