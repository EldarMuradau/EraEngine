// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core_api.h"

#include "core/math.h"
#include "core/memory.h"

#include "dx/dx_buffer.h"

#include "ai/state_machine.h"

#include "animation/skeleton.h"
#include "animation/skeleton_component.h"
#include "animation/inertial_blend_sampler.h"

#include "ecs/component.h"

namespace era_engine
{
	struct multi_mesh;
	struct ldr_render_pass;
}

namespace era_engine::animation
{
	struct ERA_CORE_API AnimationJoint
	{
		AnimationJoint() = default;

		AnimationJoint& operator=(const AnimationJoint&) = default;

		bool is_animated = false;

		uint32 first_position_keyframe = 0;
		uint32 num_position_keyframes = 0;

		uint32 first_rotation_keyframe = 0;
		uint32 num_rotation_keyframes = 0;

		uint32 first_scale_keyframe = 0;
		uint32 num_scale_keyframes = 0;
	};

	struct ERA_CORE_API AnimationClip
	{
		void edit();
		trs get_first_root_transform() const;
		trs get_last_root_transform() const;

		std::string name;
		fs::path filename;

		std::vector<float> position_timestamps;
		std::vector<float> rotation_timestamps;
		std::vector<float> scale_timestamps;

		std::vector<vec3> position_keyframes;
		std::vector<quat> rotation_keyframes;
		std::vector<vec3> scale_keyframes;

		std::vector<AnimationJoint> joints;

		AnimationJoint root_motion_joint;

		bool is_unreal_asset = false;

		float length_in_seconds = 0.0f;
		bool looping = true;
		bool bake_root_rotation_into_pose = false;
		bool bake_root_xz_translation_into_pose = false;
		bool bake_root_y_translation_into_pose = false;
	};

	struct ERA_CORE_API AnimationSkeleton
	{
		SkeletonPose sampleAnimation(const AnimationClip& clip, float time, trs* outRootMotion = 0) const;
		SkeletonPose sampleAnimation(uint32 index, float time, trs* outRootMotion = 0) const;

		std::vector<uint32> getClipsByName(const std::string& name);

		std::vector<AnimationClip> clips;
		std::vector<fs::path> files;

		Skeleton* skeleton = nullptr;
	};

	struct ERA_CORE_API AnimationInstance
	{
		AnimationInstance() { }
		AnimationInstance(const AnimationClip* clip, float startTime = 0.0f);

		void set(const AnimationClip* clip, float startTime = 0.0f);
		SkeletonPose update(const AnimationSkeleton& skeleton, float dt, trs& outDeltaRootMotion);

		bool valid() const { return clip != nullptr; }

		const AnimationClip* clip = nullptr;
		float time = 0.0f;

		trs lastRootMotion;

		bool paused = false;
		bool finished = false;
	};

	struct ERA_CORE_API AnimationBlackboard
	{
		AnimationClip* clip = nullptr;
	};

	struct ERA_CORE_API AnimationState : ai::StateBase<AnimationBlackboard>
	{
		AnimationState(ref<AnimationInstance> inst) : instance(inst)
		{
		}

		void enter(AnimationBlackboard& state) override
		{
			instance->set(state.clip);
			instance->finished = false;
		}

		void exit(AnimationBlackboard& state) override
		{
			if (instance->clip == state.clip)
			{
				instance->clip = nullptr;
			}
		}

		void pause(AnimationBlackboard& state) override
		{
			instance->paused = true;
		}

		void resume(AnimationBlackboard& state) override
		{
			instance->paused = false;
		}

	private:
		ref<AnimationInstance> instance;
	};

	struct ERA_CORE_API AnimationStateMachine
	{
		void set_state(ref<AnimationState> state, AnimationBlackboard& blackboard);

		void enter(AnimationBlackboard& blackboard);
		
		void pause(AnimationBlackboard& blackboard);

		void resume(AnimationBlackboard& blackboard);

		void update(AnimationBlackboard& blackboard);

		void update();

	private:
		ref<AnimationState> currentState = nullptr;
		std::stack<AnimationBlackboard> input;
		bool paused = false;
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