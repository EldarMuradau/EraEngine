#pragma once 

#include "core_api.h"

#include "animation/animation_clip.h"
#include "animation/skeleton.h"

namespace era_engine::animation
{
	class ERA_CORE_API AnimationPoseSampler final
	{
	public:
		AnimationPoseSampler() = default;

		void init(const Skeleton* _skeleton, std::shared_ptr<AnimationAssetClip> _animation);
		void reset();

		bool is_valid() const;
		float get_duration() const;

		std::shared_ptr<AnimationAssetClip> get_animation() const;
		const Skeleton* get_skeleton() const;

		bool sample_pose(float sample_time, SkeletonPose& target) const;

		void sample_curves(float sample_time, std::vector<float>& target) const;

		void set_joint_enabled(const std::string& joint_name, bool enabled);

	private:
		const Skeleton* skeleton = nullptr;
		std::shared_ptr<AnimationAssetClip> animation;

		std::vector<uint32> joint_mapping;
	};

	class ERA_CORE_API AnimationRootSampler final
	{
	public:
		AnimationRootSampler() = default;

		bool init(const Skeleton* skeleton, std::shared_ptr<AnimationAssetClip> _animation);
		JointTransform sample_root(float sample_time) const;

		const std::shared_ptr<AnimationAssetClip>& get_animation() const;

		bool is_valid() const;
		float get_duration() const;

	private:
		JointTransform bind_transform;

		uint32 root_joint_index = INVALID_JOINT;
		std::shared_ptr<AnimationAssetClip> animation;

		mutable JointTransform cached_root;
	};
}