#pragma once 

#include "core_api.h"

#include "asset/game_asset.h"

#include <acl/compression/track_array.h>
#include <acl/core/compressed_tracks.h>
#include <acl/decompression/decompress.h>
#include <acl/core/ansi_allocator.h>

namespace era_engine::animation
{
	class SkeletonPose;
	class JointTransform;

	class ERA_CORE_API AnimationAllocator final : public acl::ansi_allocator
	{
	public:
		AnimationAllocator() = default;
	};

	class ERA_CORE_API AnimationAssetClip final : public GameAsset
	{
	public:
		AnimationAssetClip();
		AnimationAssetClip(const AnimationAssetClip& other) = delete;
		AnimationAssetClip(AnimationAssetClip&& other) noexcept;
		AnimationAssetClip& operator=(const AnimationAssetClip& other) = delete;
		AnimationAssetClip& operator=(AnimationAssetClip&& other) noexcept;
		~AnimationAssetClip() override;

		bool is_valid() const;

		float get_duration() const;

		bool sample_joint(float sample_time, std::string_view joint_name, const JointTransform& bind_transform, JointTransform& out_transform) const;
		bool sample_joint(float sample_time, uint32 joint_index, const JointTransform& bind_transform, JointTransform& out_transform) const;
		bool sample_joints(float sample_time, SkeletonPose& out_pose) const;
		bool sample_joints(float sample_time, const SkeletonPose& default_pose, SkeletonPose& out_pose, const std::vector<uint32>& custom_joint_name_mapping) const;

		float get_sample_rate() const;
		uint32 get_num_samples_per_track() const;

		bool sample_curve(float sample_time, std::string_view curve_name, float& out_curve_value) const;
		void sample_curve(float sample_time, uint32 curve_index, float& out_curve_value) const;
		void sample_curves(float sample_time, std::vector<float>& out_curve_values) const;

		void add_curve(const std::string& name, const std::vector<float>& values);
		void remove_curve(const std::string& name);

		const std::unordered_map<std::string, uint32>& get_joint_mapping() const;
		const std::unordered_map<std::string, uint32>& get_curve_mapping() const;

		static std::string get_asset_type_impl();

		bool serialize(std::ostream& os) const override;
		bool deserialize(std::istream& is) override;

	private:
		std::unordered_map<std::string, uint32> joint_name_mapping;
		std::unordered_map<std::string, uint32> curve_name_mapping;

		acl::track_array_qvvf joints;
		acl::track_array_float1f curves;

		std::unique_ptr<AnimationAllocator> allocator;

		friend class AnimationAssetClipUtils;
	};
}