#include "animation/animation_clip_utils.h"

#include <asset/model_asset.h>

#include <geometry/mesh_builder.h>

#include <acl/compression/compress.h>
#include <acl/compression/pre_process.h>
#include <acl/compression/track_array.h>

namespace era_engine::animation
{
	static const float SAMPLE_RATE = 30.0f;

	ref<AnimationAssetClip> AnimationAssetClipUtils::make_clip(const ClipInfo& info)
	{
		ref<AnimationAssetClip> clip = make_ref<AnimationAssetClip>();

		if (info.num_samples == 0)
		{
			return clip;
		}

		for (const TrackInfo& track_info : info.tracks)
		{
			if (track_info.joint_transforms.size() != size_t(info.num_samples))
			{
				return clip;
			}
		}

		for (const CurveInfo& curve_info : info.curves)
		{
			if (curve_info.values.size() != size_t(info.num_samples))
			{
				return clip;
			}
		}

		clip->joints = acl::track_array_qvvf(*clip->allocator, uint32(info.tracks.size()));

		std::unordered_map<std::string, uint32> joint_name_mapping;

		for (uint32 track_index = 0; track_index < uint32(info.tracks.size()); ++track_index)
		{
			const TrackInfo& track_info = info.tracks[track_index];

			acl::track_desc_transformf track_description;
			track_description.output_index = track_index;
			if (track_info.parent_index == INVALID_JOINT)
			{
				track_description.parent_index = acl::k_invalid_track_index;
			}
			else
			{
				track_description.parent_index = track_info.parent_index;
			}

			track_description.shell_distance = track_info.distance;
			track_description.precision = track_info.precision;

			acl::track_qvvf qvv_track = acl::track_qvvf::make_reserve(track_description, *clip->allocator, info.num_samples, info.sample_rate);
			qvv_track.set_name(acl::string(*clip->allocator, track_info.joint_id.c_str(), track_info.joint_id.size()));
			for (uint32 sample_index = 0; sample_index != uint32(track_info.joint_transforms.size()); ++sample_index)
			{
				const trs& t = track_info.joint_transforms[sample_index];

				rtm::qvvf qvvf_value{};
				qvvf_value.rotation = rtm::quat_set(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w);
				qvvf_value.translation = rtm::vector_set(t.position.x, t.position.y, t.position.z);
				qvvf_value.scale = rtm::vector_set(t.scale.x, t.scale.y, t.scale.z);

				qvv_track[sample_index] = qvvf_value;
				clip->joints[track_index];
			}

			clip->joints[track_index] = std::move(qvv_track);
			joint_name_mapping[track_info.joint_id] = track_index;
		}

		clip->joint_name_mapping = std::move(joint_name_mapping);

		std::unordered_map<std::string, uint32> curve_name_mapping;
		clip->curves = acl::track_array_float1f(*clip->allocator, uint32(info.curves.size()));
		for (uint32 curve_index = 0; curve_index < uint32(info.curves.size()); ++curve_index)
		{
			const CurveInfo& curve_info = info.curves[curve_index];

			acl::track_desc_scalarf track_description;
			track_description.output_index = curve_index;
			track_description.precision = curve_info.precision;

			acl::track_float1f float1_track = acl::track_float1f::make_reserve(track_description, *clip->allocator, info.num_samples, info.sample_rate);
			float1_track.set_name(acl::string(*clip->allocator, curve_info.curve_id.c_str(), curve_info.curve_id.size()));
			for (uint32 sample_index = 0; sample_index < uint32(curve_info.values.size()); ++sample_index)
			{
				float1_track[sample_index] = curve_info.values[sample_index];
			}

			clip->curves[curve_index] = std::move(float1_track);
			curve_name_mapping[curve_info.curve_id] = curve_index;
		}

		clip->curve_name_mapping = std::move(curve_name_mapping);

		return clip;
	}

	ref<AnimationAssetClip> AnimationAssetClipUtils::make_clip(AnimationClipAssetImportData& anim_import_data, const Skeleton* skeleton, uint32 flags/* = 0*/)
	{
		const float sample_rate = SAMPLE_RATE;

		ClipInfo clip_info;
		clip_info.num_samples = anim_import_data.poses.size();
		clip_info.sample_rate = sample_rate;
		clip_info.curves = {};

		const float timestamp = 1.0f / clip_info.sample_rate;

		clip_info.tracks.resize(skeleton->joints.size());
		for (uint32 current_sample = 0; current_sample < clip_info.num_samples; ++current_sample)
		{
			const SkeletonPose& current_pose = anim_import_data.poses.at(current_sample);

			for (uint32 i = 0; i < skeleton->joints.size(); ++i)
			{
				TrackInfo& track_info = clip_info.tracks[i];
				track_info.joint_id = skeleton->joints[i].name;
				track_info.parent_index = skeleton->joints[i].parent_id == INVALID_JOINT ? acl::k_invalid_track_index : skeleton->joints[i].parent_id;
				if (flags & mesh_creation_flags_yzx_to_xyz &&
					track_info.parent_index == acl::k_invalid_track_index)
				{
					track_info.joint_transforms.emplace_back(current_pose.get_joint_transform(i).get_transform() * trs { vec3::zero, euler_to_quat(vec3(0.0f, -M_PI / 2.0f, 0.0f)), vec3(1.0f) });
				}
				else
				{
					track_info.joint_transforms.emplace_back(current_pose.get_joint_transform(i).get_transform());
				}
			}
		}

		return make_clip(clip_info);
	}

	static void generate_root_motion_in_place(
		AnimationClipAssetImportData& anim_data,
		uint32 hips_joint_id)
	{
		if (anim_data.poses.empty())
		{
			return;
		}

		size_t num_poses = anim_data.poses.size();
		size_t old_joint_count = anim_data.poses[0].size();
		size_t new_joint_count = old_joint_count + 1;

		uint32 root_motion_joint_id = 0;
		uint32 new_hips_joint_id = hips_joint_id + 1;

		std::vector<SkeletonPose> new_poses;
		new_poses.reserve(num_poses);

		for (size_t i = 0; i < num_poses; ++i)
		{
			SkeletonPose new_pose((uint32)new_joint_count);

			for (uint32 j = 1; j < old_joint_count; ++j)
			{
				const trs& joint_transform = anim_data.poses[i].get_joint_transform(j).get_transform();
				new_pose.set_joint_transform(JointTransform(joint_transform), j + 1);
			}

			const trs& hips_transform = anim_data.poses[i].get_joint_transform(hips_joint_id).get_transform();

			vec3 hips_euler = quat_to_euler(hips_transform.rotation);

			trs root_motion;
			root_motion.position = vec3(hips_transform.position.x, 0, hips_transform.position.z);
			root_motion.rotation = euler_to_quat(vec3(0, hips_euler.y, 0));
			root_motion.scale = vec3(1.0f);

			trs root_inverse = invert(root_motion);
			trs hips_local = root_inverse * hips_transform;

			new_pose.set_joint_transform(JointTransform(root_motion), root_motion_joint_id);
			new_pose.set_joint_transform(JointTransform(hips_local), new_hips_joint_id);

			new_poses.push_back(std::move(new_pose));
		}

		anim_data.poses = std::move(new_poses);
	}

	std::vector<ref<AnimationAssetClip>> AnimationAssetClipUtils::import_animations(std::vector<AnimationClipAssetImportData>& animations_to_import,
		const ref<Skeleton>& skeleton,
		const fs::path& file,
		uint32 flags/* = 0*/)
	{
		std::vector<ref<AnimationAssetClip>> result;
		result.reserve(animations_to_import.size());

		GameAssetsProvider provider;

		uint32 anim_index = 0;

		std::vector<JobHandle> handles;
		handles.reserve(animations_to_import.size());

		for (auto& anim : animations_to_import)
		{
			fs::path clip_path = file.parent_path();
			clip_path.append("animations");
			clip_path.append("animation_clip" + std::to_string(anim_index));

			if (!fs::exists(fs::path(clip_path.string() + AssetExtension<AnimationAssetClip>::get_asset_type())))
			{
				if (flags & mesh_creation_flags_generate_root_motion)
				{
					generate_root_motion_in_place(anim, 0);
				}

				ref<AnimationAssetClip> animation_clip = AnimationAssetClipUtils::make_clip(anim, skeleton.get(), flags);

				handles.emplace_back(provider.save_game_asset_to_file_async<AnimationAssetClip>(clip_path, animation_clip.get()));

				result.push_back(std::move(animation_clip));
			}
			anim_index++;
		}

		for (JobHandle handle : handles)
		{
			handle.wait_for_completion();
		}

		return result;
	}
}