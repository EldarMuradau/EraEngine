// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "animation/animation.h"
#include "animation/skinning.h"

#include "core/memory.h"
#include "core/random.h"
#include "core/imgui.h"
#include "core/string.h"

#include "geometry/mesh.h"

#include "dx/dx_context.h"

#include "rendering/debug_visualization.h"

#include <rttr/registration>

#include <algorithm>

namespace era_engine::animation
{
	const char* limb_type_names[] =
	{
		"Unknown",

		"Torso",
		"Head",

		"Upper arm right",
		"Lower arm right",
		"Hand right",

		"Upper arm left",
		"Lower arm left",
		"Hand left",

		"Upper leg right",
		"Lower leg right",
		"Foot right",

		"Upper leg left",
		"Lower leg left",
		"Foot left",
	};

	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<AnimationComponent>("AnimationComponent")
			.constructor<ref<Entity::EcsData>>();
	}

	AnimationComponent::AnimationComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
	}

	AnimationComponent::~AnimationComponent()
	{
	}

	void AnimationComponent::activate_inertial_blend()
	{
		const SkeletonComponent* skeleton_component = get_entity().get_component<SkeletonComponent>();

		inertial_sampler = make_ref<InertialBlendSampler>(skeleton_component->skeleton.get());
		inertial_blend_inited = true;
	}

	void AnimationComponent::trigger_reset_inertial_blend(const SkeletonPose& reset_pose, float _blend_time)
	{
		blend_time = _blend_time;
		inertial_sampler->reset_blend_time(blend_time);
		inertial_sampler->init_inertial_blend(reset_pose);
	}
}