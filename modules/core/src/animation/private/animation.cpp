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
	static void scale_keyframes(AnimationClip& clip, AnimationJoint& joint, float scale)
	{
		for (uint32 keyID = 0; keyID < joint.num_position_keyframes; ++keyID)
		{
			clip.position_keyframes[joint.first_position_keyframe + keyID] *= scale;
		}
		for (uint32 keyID = 0; keyID < joint.num_scale_keyframes; ++keyID)
		{
			clip.scale_keyframes[joint.first_scale_keyframe + keyID] *= scale;
		}
	}

	static vec3 samplePosition(const AnimationClip& clip, const AnimationJoint& animJoint, float time)
	{
		if (time >= clip.length_in_seconds)
		{
			return clip.position_keyframes[animJoint.first_position_keyframe + animJoint.num_position_keyframes - 1];
		}

		if (animJoint.num_position_keyframes == 1)
		{
			return clip.position_keyframes[animJoint.first_position_keyframe];
		}

		uint32 firstKeyframeIndex = -1;
		for (uint32 i = 0; i < animJoint.num_position_keyframes - 1; ++i)
		{
			uint32 j = i + animJoint.first_position_keyframe;
			if (time < clip.position_timestamps[j + 1])
			{
				firstKeyframeIndex = j;
				break;
			}
		}
		ASSERT(firstKeyframeIndex != -1);

		uint32 secondKeyframeIndex = firstKeyframeIndex + 1;

		float t = inverse_lerp(clip.position_timestamps[firstKeyframeIndex], clip.position_timestamps[secondKeyframeIndex], time);

		vec3 a = clip.position_keyframes[firstKeyframeIndex];
		vec3 b = clip.position_keyframes[secondKeyframeIndex];

		return lerp(a, b, t);
	}

	static quat sampleRotation(const AnimationClip& clip, const AnimationJoint& animJoint, float time)
	{
		if (time >= clip.length_in_seconds)
		{
			return clip.rotation_keyframes[animJoint.first_rotation_keyframe + animJoint.num_rotation_keyframes - 1];
		}

		if (animJoint.num_rotation_keyframes == 1)
		{
			return clip.rotation_keyframes[animJoint.first_rotation_keyframe];
		}

		uint32 firstKeyframeIndex = -1;
		for (uint32 i = 0; i < animJoint.num_rotation_keyframes - 1; ++i)
		{
			uint32 j = i + animJoint.first_rotation_keyframe;
			if (time < clip.rotation_timestamps[j + 1])
			{
				firstKeyframeIndex = j;
				break;
			}
		}
		ASSERT(firstKeyframeIndex != -1);

		uint32 secondKeyframeIndex = firstKeyframeIndex + 1;

		float t = inverse_lerp(clip.rotation_timestamps[firstKeyframeIndex], clip.rotation_timestamps[secondKeyframeIndex], time);

		quat a = clip.rotation_keyframes[firstKeyframeIndex];
		quat b = clip.rotation_keyframes[secondKeyframeIndex];

		if (dot(a.v4, b.v4) < 0.f)
		{
			b.v4 *= -1.f;
		}

		return lerp(a, b, t);
	}

	static vec3 sampleScale(const AnimationClip& clip, const AnimationJoint& animJoint, float time)
	{
		if (time >= clip.length_in_seconds)
			return clip.scale_keyframes[animJoint.first_scale_keyframe + animJoint.num_scale_keyframes - 1];

		if (animJoint.num_scale_keyframes == 1)
			return clip.scale_keyframes[animJoint.first_scale_keyframe];

		if (!clip.scale_timestamps.size())
			return vec3(1.0f);

		uint32 firstKeyframeIndex = -1;
		for (uint32 i = 0; i < animJoint.num_scale_keyframes - 1; ++i)
		{
			uint32 j = i + animJoint.first_scale_keyframe;

			if (time < clip.scale_timestamps[j + 1])
			{
				firstKeyframeIndex = j;
				break;
			}
		}
		ASSERT(firstKeyframeIndex != -1);

		uint32 secondKeyframeIndex = firstKeyframeIndex + 1;

		float t = inverse_lerp(clip.scale_timestamps[firstKeyframeIndex], clip.scale_timestamps[secondKeyframeIndex], time);

		vec3 a = clip.scale_keyframes[firstKeyframeIndex];
		vec3 b = clip.scale_keyframes[secondKeyframeIndex];

		return lerp(a, b, t);
	}

	SkeletonPose AnimationSkeleton::sampleAnimation(const AnimationClip& clip, float time, trs* outRootMotion) const
	{
		ASSERT(skeleton != nullptr);
		ASSERT(clip.joints.size() == skeleton->joints.size());

		SkeletonPose result_pose = SkeletonPose(skeleton->joints.size());

		time = clamp(time, 0.f, clip.length_in_seconds);

		uint32 numJoints = (uint32)skeleton->joints.size();
		for (uint32 i = 0; i < numJoints; ++i)
		{
			const AnimationJoint& animJoint = clip.joints[i];
			JointTransform joint_transform;

			if (animJoint.is_animated)
			{
				trs transform;
				transform.position = samplePosition(clip, animJoint, time);
				transform.rotation = sampleRotation(clip, animJoint, time);
				transform.scale = sampleScale(clip, animJoint, time);

				joint_transform.set_transform(transform);
			}
			else
			{
				joint_transform.set_transform(trs::identity);
			}

			result_pose.set_joint_transform(joint_transform, i);
		}

		trs rootMotion;
		if (clip.root_motion_joint.is_animated)
		{
			rootMotion.position = samplePosition(clip, clip.root_motion_joint, time);
			rootMotion.rotation = sampleRotation(clip, clip.root_motion_joint, time);
			rootMotion.scale = sampleScale(clip, clip.root_motion_joint, time);
		}
		else
		{
			rootMotion = trs::identity;
		}

		if (outRootMotion)
		{
			if (clip.bake_root_rotation_into_pose)
			{
				result_pose.set_joint_transform(JointTransform(trs(0.f, rootMotion.rotation) * result_pose.get_joint_transform(0).get_transform()), 0);
				rootMotion.rotation = quat::identity;
			}

			if (clip.bake_root_xz_translation_into_pose)
			{
				result_pose.set_joint_translation(result_pose.get_joint_translation(0)
					+ vec3(rootMotion.position.x, 0.0f, 0.0f) + vec3(0.0f, 0.0f, rootMotion.position.z), 0);
				rootMotion.position.x = 0.f;
				rootMotion.position.z = 0.f;
			}

			if (clip.bake_root_y_translation_into_pose)
			{
				result_pose.set_joint_translation(result_pose.get_joint_translation(0)
					+ vec3(rootMotion.position.x, 0.0f, 0.0f) + vec3(0.0f, rootMotion.position.y, 0.0f), 0);
				rootMotion.position.y = 0.f;
			}

			*outRootMotion = rootMotion;
		}
		else
		{
			result_pose.set_joint_transform(JointTransform(rootMotion * result_pose.get_joint_transform(0).get_transform()), 0);
		}

		if (clip.is_unreal_asset)
		{
			result_pose.set_joint_rotation(result_pose.get_joint_rotation(0) * euler_to_quat(vec3(0.0f, -M_PI / 2.0f, 0.0f)), 0);
		}

		return result_pose;
	}

	SkeletonPose AnimationSkeleton::sampleAnimation(uint32 index, float time, trs* outRootMotion) const
	{
		return sampleAnimation(clips[index], time, outRootMotion);
	}

	std::vector<uint32> AnimationSkeleton::getClipsByName(const std::string& name)
	{
		std::vector<uint32> result;
		for (uint32 i = 0; i < (uint32)clips.size(); ++i)
		{
			if (clips[i].name == name)
			{
				result.push_back(i);
			}
		}
		return result;
	}

	void AnimationClip::edit()
	{
		ImGui::Checkbox("Bake root rotation into pose", &bake_root_rotation_into_pose);
		ImGui::Checkbox("Bake xz translation into pose", &bake_root_xz_translation_into_pose);
		ImGui::Checkbox("Bake y translation into pose", &bake_root_y_translation_into_pose);
	}

	trs AnimationClip::get_first_root_transform() const
	{
		if (root_motion_joint.is_animated)
		{
			trs t;
			t.position = position_keyframes[root_motion_joint.first_position_keyframe];
			t.rotation = rotation_keyframes[root_motion_joint.first_rotation_keyframe];
			t.scale = scale_keyframes[root_motion_joint.first_scale_keyframe];

			if (bake_root_rotation_into_pose)
			{
				t.rotation = quat::identity;
			}
			if (bake_root_xz_translation_into_pose)
			{
				t.position.x = 0.f;
				t.position.z = 0.f;
			}
			if (bake_root_y_translation_into_pose)
			{
				t.position.y = 0.f;
			}

			return t;
		}
		return trs::identity;
	}

	trs AnimationClip::get_last_root_transform() const
	{
		if (root_motion_joint.is_animated)
		{
			trs t;
			t.position = position_keyframes[root_motion_joint.first_position_keyframe + root_motion_joint.num_position_keyframes - 1];
			t.rotation = rotation_keyframes[root_motion_joint.first_rotation_keyframe + root_motion_joint.num_rotation_keyframes - 1];
			t.scale = scale_keyframes[root_motion_joint.first_scale_keyframe + root_motion_joint.num_scale_keyframes - 1];

			if (bake_root_rotation_into_pose)
			{
				t.rotation = quat::identity;
			}
			if (bake_root_xz_translation_into_pose)
			{
				t.position.x = 0.f;
				t.position.z = 0.f;
			}
			if (bake_root_y_translation_into_pose)
			{
				t.position.y = 0.f;
			}

			return t;
		}
		return trs::identity;
	}

	AnimationInstance::AnimationInstance(const AnimationClip* clip, float startTime)
	{
		set(clip, startTime);
	}

	void AnimationInstance::set(const AnimationClip* clip, float startTime)
	{
		this->clip = clip;
		time = startTime;
		lastRootMotion = clip->get_first_root_transform();
	}

	SkeletonPose AnimationInstance::update(const AnimationSkeleton& skeleton, float dt, trs& outDeltaRootMotion)
	{
		if (paused)
		{
			return SkeletonPose();
		}

		if (valid())
		{
			time += dt;
			if (time >= clip->length_in_seconds)
			{
				if (clip->looping)
				{
					time = fmod(time, clip->length_in_seconds);
					lastRootMotion = clip->get_first_root_transform();
				}
				else
				{
					time = clip->length_in_seconds;
					finished = true;
				}
			}

			trs rootMotion;
			SkeletonPose result_pose = skeleton.sampleAnimation(*clip, time, &rootMotion);

			outDeltaRootMotion = invert(lastRootMotion) * rootMotion;
			lastRootMotion = rootMotion;
			return result_pose;
		}
		return SkeletonPose();
	}

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

	void AnimationStateMachine::set_state(ref<AnimationState> state, AnimationBlackboard& blackboard)
	{
		if (currentState)
		{
			currentState->exit(blackboard);
		}

		currentState = state;

		if (currentState)
		{
			currentState->enter(blackboard);

			if (paused)
			{
				currentState->pause(blackboard);
			}
		}
	}

	void AnimationStateMachine::enter(AnimationBlackboard& blackboard)
	{
		if (currentState)
		{
			currentState->enter(blackboard);
			input.push(blackboard);
			if (paused)
			{
				currentState->pause(blackboard);
			}
		}
	}

	void AnimationStateMachine::pause(AnimationBlackboard& blackboard)
	{
		paused = true;

		if (currentState)
		{
			currentState->pause(blackboard);
		}
	}

	void AnimationStateMachine::resume(AnimationBlackboard& blackboard)
	{
		paused = false;

		if (currentState)
			currentState->resume(blackboard);
	}

	void AnimationStateMachine::update(AnimationBlackboard& blackboard)
	{
		if (paused)
			return;

		if (currentState)
		{
			input.push(blackboard);
			currentState->update(blackboard);
		}
	}

	void AnimationStateMachine::update()
	{
		if (paused)
			return;

		if (currentState && input.size())
		{
			currentState->enter(input.top());
			input.pop();
		}
	}

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