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
	struct ERA_CORE_API AnimationSkeletonImportData
	{
		SkeletonPose sampleAnimation(const AnimationClipImportData& clip, float time, trs* outRootMotion = 0) const;
		SkeletonPose sampleAnimation(uint32 index, float time, trs* outRootMotion = 0) const;

		std::vector<uint32> getClipsByName(const std::string& name);

		std::vector<AnimationClipImportData> clips;
		std::vector<fs::path> files;

		Skeleton* skeleton = nullptr;
	};

	struct ERA_CORE_API AnimationInstanceImportData
	{
		AnimationInstanceImportData() { }
		AnimationInstanceImportData(const AnimationClipImportData* clip, float startTime = 0.0f);

		void set(const AnimationClipImportData* clip, float startTime = 0.0f);
		SkeletonPose update(const AnimationSkeletonImportData& skeleton, float dt, trs& outDeltaRootMotion);

		bool valid() const { return clip != nullptr; }

		const AnimationClipImportData* clip = nullptr;
		float time = 0.0f;

		trs lastRootMotion;

		bool paused = false;
		bool finished = false;
	};

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