#include "animation/skeleton.h"

#include "asset/io.h"

#include "core/string.h"
#include "core/io/binary_data.h"
#include "core/log.h"

#include <geometry/mesh_builder.h>

#include <sstream>

namespace era_engine::animation
{
	static void pretty_print(const Skeleton& skeleton, uint32 parent, uint32 indent)
	{
		for (uint32 i = 0; i < (uint32)skeleton.joints.size(); ++i)
		{
			if (skeleton.joints[i].parent_id == parent)
			{
				std::cout << std::string(indent, ' ') << skeleton.joints[i].name << '\n';
				pretty_print(skeleton, i, indent + 1);
			}
		}
	}

	JointTransform::JointTransform(const trs& initial_transform)
	{
		local_transform = initial_transform;
	}

	void JointTransform::set_translation(const vec3& new_translation)
	{
		local_transform.position = new_translation;
	}

	const vec3& JointTransform::get_translation() const
	{
		return local_transform.position;
	}

	void JointTransform::set_scale(const vec3& new_scale)
	{
		local_transform.scale = new_scale;
	}

	const vec3& JointTransform::get_scale() const
	{
		return local_transform.scale;
	}

	void JointTransform::set_rotation(const quat& new_rotation)
	{
		local_transform.rotation = new_rotation;
	}

	const quat& JointTransform::get_rotation() const
	{
		return local_transform.rotation;
	}

	void JointTransform::set_transform(const trs& new_transform)
	{
		local_transform = new_transform;
	}

	const trs& JointTransform::get_transform() const
	{
		return local_transform;
	}

	SkeletonPose::SkeletonPose(uint32 joints_size)
	{
		local_transforms.resize(joints_size);
	}

	SkeletonPose::SkeletonPose(const std::vector<JointTransform>& _local_transforms)
		: local_transforms(_local_transforms)
	{
	}

	bool SkeletonPose::is_valid() const
	{
		return local_transforms.size() > 0;
	}

	uint32 SkeletonPose::size() const
	{
		return static_cast<uint32>(local_transforms.size());
	}

	void SkeletonPose::set_joint_transform(const JointTransform& new_joint_transform, uint32 joint_id)
	{
		local_transforms[joint_id] = new_joint_transform;
	}

	void SkeletonPose::set_joint_translation(const vec3& new_translation, uint32 joint_id)
	{
		local_transforms[joint_id].set_translation(new_translation);
	}

	void SkeletonPose::set_joint_scale(const vec3& new_scale, uint32 joint_id)
	{
		local_transforms[joint_id].set_scale(new_scale);
	}

	void SkeletonPose::set_joint_rotation(const quat& new_rotaiton, uint32 joint_id)
	{
		local_transforms[joint_id].set_rotation(new_rotaiton);
	}

	const vec3& SkeletonPose::get_joint_translation(uint32 joint_id) const
	{
		return local_transforms[joint_id].get_translation();
	}

	const vec3& SkeletonPose::get_joint_scale(uint32 joint_id) const
	{
		return local_transforms[joint_id].get_scale();
	}

	const JointTransform& SkeletonPose::get_joint_transform(uint32 joint_id) const
	{
		return local_transforms[joint_id];
	}

	const quat& SkeletonPose::get_joint_rotation(uint32 joint_id) const
	{
		return local_transforms[joint_id].get_rotation();
	}

	void Skeleton::analyze_joints()
	{
		for (uint32 jointID = 0; jointID < (uint32)joints.size(); ++jointID)
		{
			SkeletonJoint& j = joints[jointID];

			std::string name = j.name;
			std::transform(name.begin(), name.end(), name.begin(), ::tolower);

			LimbType c = limb_type_none;

			bool left = contains(name, "left") || 
				ends_with(name, ".l") || 
				ends_with(name, "_l");

			if (contains(name, "spine") ||
				contains(name, "hip") ||
				contains(name, "rib") ||
				contains(name, "pelvis") ||
				contains(name, "shoulder") ||
				contains(name, "clavicle"))
			{
				c = limb_type_torso;
			}
			else if (contains(name, "head") ||
				contains(name, "neck"))
			{
				c = limb_type_head;
			}
			else if (contains(name, "arm"))
			{
				LimbType parentType = (j.parent_id != INVALID_JOINT) ? joints[j.parent_id].limb_type : limb_type_none;

				if (contains(name, "lower") ||
					contains(name, "lo") ||
					contains(name, "fore"))
				{
					c = left ? limb_type_lower_arm_left : limb_type_lower_arm_right;
				}
				else if (contains(name, "upper") ||
					contains(name, "up"))
				{
					c = left ? limb_type_upper_arm_left : limb_type_upper_arm_right;
				}
				else if (parentType == limb_type_torso)
				{
					c = left ? limb_type_upper_arm_left : limb_type_upper_arm_right;
				}
				else
				{
					c = left ? limb_type_lower_arm_left : limb_type_lower_arm_right;
				}
			}
			else if (contains(name, "hand") ||
				contains(name, "finger") ||
				contains(name, "thumb") ||
				contains(name, "index") ||
				contains(name, "middle") ||
				contains(name, "ring") ||
				contains(name, "pinky"))
			{
				c = left ? limb_type_hand_left : limb_type_hand_right;
			}
			else if (contains(name, "leg") ||
				contains(name, "thigh") ||
				contains(name, "shin") ||
				contains(name, "calf"))
			{
				LimbType parentType = (j.parent_id != INVALID_JOINT) ? joints[j.parent_id].limb_type : limb_type_none;

				if (contains(name, "lower") ||
					contains(name, "lo") ||
					contains(name, "shin") ||
					contains(name, "calf"))
				{
					c = left ? limb_type_lower_leg_left : limb_type_lower_leg_right;
				}
				else if (contains(name, "upper") ||
					contains(name, "up") ||
					contains(name, "thigh"))
				{
					c = left ? limb_type_upper_leg_left : limb_type_upper_leg_right;
				}
				else if (parentType == limb_type_torso)
				{
					c = left ? limb_type_upper_leg_left : limb_type_upper_leg_right;
				}
				else
				{
					c = left ? limb_type_lower_leg_left : limb_type_lower_leg_right;
				}
			}
			else if (contains(name, "foot") ||
				contains(name, "toe") ||
				contains(name, "ball"))
			{
				c = left ? limb_type_foot_left : limb_type_foot_right;
			}

			j.limb_type = c;
			j.ik = contains(name, "ik");
		}
	}

	void Skeleton::pretty_print_hierarchy() const
	{
		pretty_print(*this, INVALID_JOINT, 0);
	}

	std::string Skeleton::get_asset_type_impl()
	{
		return std::string(".eskeleton");
	}

	void Skeleton::set_joint_transform(const trs& new_transform, uint32 joint_id)
	{
		default_local_transforms[joint_id].set_transform(new_transform);
	}

	void Skeleton::set_joint_rotation(const quat& new_rotaiton, uint32 joint_id)
	{
		default_local_transforms[joint_id].set_rotation(new_rotaiton);
	}

	const trs& Skeleton::get_joint_transform(uint32 joint_id) const
	{
		return default_local_transforms[joint_id].get_transform();
	}

	const quat& Skeleton::get_joint_rotation(uint32 joint_id) const
	{
		return default_local_transforms[joint_id].get_rotation();
	}

	uint32 Skeleton::get_joint_index(std::string_view name) const
	{
		for (uint32 i = 0; i < joints.size(); ++i)
		{
			if (joints[i].name == name)
			{
				return i;
			}
		}
		return INVALID_JOINT;
	}

	const SkeletonPose& Skeleton::get_default_pose() const
	{
		if (default_pose.local_transforms.size() == 0)
		{
			default_pose = SkeletonPose(joints.size());
			for (uint32 i = 0; i < joints.size(); ++i)
			{
				default_pose.set_joint_transform(JointTransform(default_local_transforms.at(i)), i);
			}
		}

		return default_pose;
	}

	void Skeleton::set_joint_translation(const vec3& new_translation, uint32 joint_id)
	{
		default_local_transforms[joint_id].set_translation(new_translation);
	}

	const vec3& Skeleton::get_joint_translation(uint32 joint_id) const
	{
		return default_local_transforms[joint_id].get_translation();
	}

	struct SkeletonSerializationData
	{
		std::vector<SkeletonJoint> joints;
		std::vector<JointTransform> default_local_transforms;
		std::unordered_map<std::string, uint32> name_to_joint_id;

		ERA_BINARY_SERIALIZE(joints, default_local_transforms, name_to_joint_id)
	};

	bool Skeleton::serialize(std::ostream& os) const
	{
		SkeletonSerializationData data;

		data.name_to_joint_id.insert(std::begin(name_to_joint_id), std::end(name_to_joint_id));
		data.joints = joints;
		data.default_local_transforms = default_local_transforms;

		const BinaryDataArchive& serialized_data = BinarySerializer::serialize(data);

		try
		{
			if (!IO::write_value(os, serialized_data))
			{
				return false;
			}
		}
		catch (...)
		{
			LOG_ERROR("Exception thrown while serializing asset!");
			return false;
		}

		return true;
	}

	bool Skeleton::deserialize(std::istream& is)
	{
		BinaryDataArchive serialized_data;

		IO::read_value(is, serialized_data);

		SkeletonSerializationData data{};
		if (BinarySerializer::deserialize(BinaryData(serialized_data), data) != serialized_data.size())
		{
			return false;
		}

		name_to_joint_id = std::move(data.name_to_joint_id);
		joints = std::move(data.joints);
		default_local_transforms = std::move(data.default_local_transforms);

		get_default_pose();

		return true;
	}

	std::vector<ref<Skeleton>> SkeletonImportUtils::import_skeletons(std::vector<SkeletonAssetImportData>& skeletons_to_import, const fs::path& file, uint32 flags)
	{
		std::vector<ref<Skeleton>> result;
		result.reserve(skeletons_to_import.size());

		std::vector<JobHandle> handles;
		handles.reserve(skeletons_to_import.size());

		GameAssetsProvider provider;

		// Load skeleton
		if (!skeletons_to_import.empty() && (flags & mesh_creation_flags_with_skin))
		{
			uint32 skeleton_index = 0;
			for (SkeletonAssetImportData& skeleton_asset : skeletons_to_import)
			{
				fs::path skeleton_path = file.parent_path();
				skeleton_path.append("skeletons");
				skeleton_path.append("skeleton" + std::to_string(skeleton_index));

				if (!fs::exists(fs::path(skeleton_path.string() + AssetExtension<Skeleton>::get_asset_type())))
				{
					ref<Skeleton> imported_skeleton = make_ref<Skeleton>();
					result.emplace_back(imported_skeleton);

					imported_skeleton->joints = std::move(skeleton_asset.joints);

					imported_skeleton->default_local_transforms.reserve(imported_skeleton->joints.size());
					for (size_t i = 0; i < imported_skeleton->joints.size(); ++i)
					{
						SkeletonJoint& joint = imported_skeleton->joints[i];
						JointTransform joint_transform;

						if (joint.parent_id > imported_skeleton->joints.size() || joint.parent_id == INVALID_JOINT)
						{
							if (flags & mesh_creation_flags_yzx_to_xyz)
							{
								joint_transform.set_transform(mat4_to_trs(joint.bind_transform) * trs { vec3::zero, euler_to_quat(vec3(0.0f, M_PI / 2.0f, 0.0f)), vec3(1.0f) });
							}
							else
							{
								joint_transform.set_transform(mat4_to_trs(joint.bind_transform));
							}
						}
						else
						{
							const auto& parent_bind = imported_skeleton->joints[joint.parent_id].bind_transform;
							joint_transform.set_transform(mat4_to_trs(invert(parent_bind) * joint.bind_transform));
						}

						imported_skeleton->default_local_transforms.push_back(joint_transform);
					}

					imported_skeleton->name_to_joint_id = std::move(skeleton_asset.name_to_joint_id);
					imported_skeleton->analyze_joints();

					handles.emplace_back(provider.save_game_asset_to_file_async<Skeleton>(skeleton_path, imported_skeleton.get()));
					skeleton_index++;
				}
			}
		}

		for (JobHandle handle : handles)
		{
			handle.wait_for_completion();
		}

		return result;
	}
}