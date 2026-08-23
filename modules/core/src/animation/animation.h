#pragma once

#include "core_api.h"

#include "core/math.h"
#include "core/memory.h"

#include "dx/dx_buffer.h"

#include "ai/state_machine.h"

#include "animation/animation_common_data.h"
#include "animation/skeleton.h"
#include "animation/skeleton_component.h"
#include "animation/inertial_blend_sampler.h"

#include "ecs/component.h"

namespace era_engine
{
	class MultiMesh;
	struct ldr_render_pass;
}

namespace era_engine::animation
{
	class ERA_CORE_API AnimationComponent : public Component
	{
	public:
		AnimationComponent(ref<Entity::EcsData> _data);
		~AnimationComponent() override;

		void activate_inertial_blend();
		void trigger_reset_inertial_blend(const SkeletonPose& reset_pose, float _blend_time);

		ERA_VIRTUAL_REFLECT(Component)

	public:
		ref<AnimationAssetClip> current_animation;
		float current_anim_position = 0.0f;

		SkeletonPose current_animation_pose;

		bool play = true;
		bool update_skeleton = true;
		bool loop = false;

		// Render data
		dx_vertex_buffer_group_view current_vertex_buffer;
		dx_vertex_buffer_group_view prev_frame_vertex_buffer;

		trs* current_global_transforms = nullptr;

		ref<InertialBlendSampler> inertial_sampler;
		float blend_time = 0.16f;

		bool inertial_blend_inited = false;
	};
}