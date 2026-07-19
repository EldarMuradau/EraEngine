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

	ref<AnimationAssetClip> AnimationAssetClipUtils::make_clip(const AnimationSkeletonImportData& old_anim_skeleton, uint32 clip_index, const Skeleton* skeleton)
	{
		const float sample_rate = SAMPLE_RATE;
		const AnimationClipImportData& old_type_clip = old_anim_skeleton.clips.at(clip_index);

		ClipInfo clip_info;
		clip_info.num_samples = std::lrintf(old_type_clip.length_in_seconds * sample_rate);
		clip_info.sample_rate = sample_rate;
		clip_info.curves = {};

		const float timestamp = 1.0f / clip_info.sample_rate;

		clip_info.tracks.resize(skeleton->joints.size());
		for (uint32 current_sample = 0; current_sample < clip_info.num_samples; ++current_sample)
		{
			SkeletonPose current_pose = old_anim_skeleton.sampleAnimation(old_type_clip, current_sample * timestamp);

			for (uint32 i = 0; i < skeleton->joints.size(); ++i)
			{
				TrackInfo& track_info = clip_info.tracks[i];
				track_info.joint_id = skeleton->joints[i].name;
				track_info.parent_index = skeleton->joints[i].parent_id == INVALID_JOINT ? acl::k_invalid_track_index : skeleton->joints[i].parent_id;
				if (old_type_clip.is_unreal_asset &&
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

	std::vector<ref<AnimationAssetClip>> AnimationAssetClipUtils::import_animations(std::vector<AnimationAssetImportData>& animations_to_import,
		const ref<Skeleton>& skeleton,
		const fs::path& file,
		uint32 flags/* = 0*/)
	{
		std::vector<ref<AnimationAssetClip>> result;
		result.reserve(animations_to_import.size());

		GameAssetsProvider provider;

		AnimationSkeletonImportData animation_skeleton;
		animation_skeleton.clips.reserve(animations_to_import.size());
		animation_skeleton.skeleton = skeleton.get();

		uint32 anim_index = 0;
		uint32 clips_loaded = 0;

		std::vector<JobHandle> handles;
		handles.reserve(animations_to_import.size());

		for (auto& anim : animations_to_import)
		{
			fs::path clip_path = file.parent_path();
			clip_path.append("animations");
			clip_path.append("animation_clip" + std::to_string(anim_index));

			if (!fs::exists(fs::path(clip_path.string() + AssetExtension<AnimationAssetClip>::get_asset_type())))
			{
				AnimationAssetImportData& in = anim;

				AnimationClipImportData& clip = animation_skeleton.clips.emplace_back();
				clip.name = std::move(in.name);
				clip.is_unreal_asset = flags & mesh_creation_flags_unreal_asset;
				clip.filename = clip_path;
				clip.length_in_seconds = in.duration;
				clip.joints.resize(skeleton->joints.size(), {});

				clip.position_keyframes = std::move(in.position_keyframes);
				clip.position_timestamps = std::move(in.position_timestamps);
				clip.rotation_keyframes = std::move(in.rotation_keyframes);
				clip.rotation_timestamps = std::move(in.rotation_timestamps);
				clip.scale_keyframes = std::move(in.scale_keyframes);
				clip.scale_timestamps = std::move(in.scale_timestamps);

				for (auto& [name, joint] : in.joints)
				{
					auto it = skeleton->name_to_joint_id.find(name);
					if (it != skeleton->name_to_joint_id.end())
					{
						AnimationJointImportData& j = clip.joints[it->second];
						j = joint;
					}
				}

				clip.root_motion_joint = clip.joints[0];

				ref<AnimationAssetClip> animation_clip = AnimationAssetClipUtils::make_clip(animation_skeleton, clips_loaded, skeleton.get());

				handles.emplace_back(provider.save_game_asset_to_file_async<AnimationAssetClip>(clip_path, animation_clip.get()));
				clips_loaded++;

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