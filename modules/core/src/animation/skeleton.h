#pragma once 

#include "core_api.h"

#include "animation/animation_common_data.h"
#include "animation/animation_clip.h"

#include "asset/game_asset.h"
#include "asset/model_asset.h"

#include "core/serialization/binary_serializer.h"
#include "core/math.h"

#include <array>

namespace era_engine::animation
{
	extern const char* limb_type_names[limb_type_count];
	extern const vec3 limb_type_colors[limb_type_count];

	class ERA_CORE_API JointTransform
	{
	public:
		JointTransform() = default;
		JointTransform(const trs& initial_transform);

		void set_translation(const vec3& new_translation);
		const vec3& get_translation() const;

		void set_scale(const vec3& new_scale);
		const vec3& get_scale() const;

		void set_rotation(const quat& new_rotation);
		const quat& get_rotation() const;

		void set_transform(const trs& new_transform);
		const trs& get_transform() const;

		ERA_BINARY_SERIALIZE(local_transform)

	private:
		trs local_transform = trs::identity; // In parent space.
	};

	struct ERA_CORE_API LimbDimensions final
	{
		float minY = 0.0;
		float maxY = 0.0f;
		float radius = 0.0f;
		float xOffset = 0.0f;
		float zOffset = 0.0f;

		ERA_BINARY_SERIALIZE(minY, maxY, radius, xOffset, zOffset)
	};

	struct ERA_CORE_API SkeletonLimb final
	{
		uint32 representative_joint = INVALID_JOINT;
		LimbDimensions dimensions;

		ERA_BINARY_SERIALIZE(representative_joint, dimensions)
	};

	class ERA_CORE_API SkeletonPose
	{
	public:
		SkeletonPose() = default;
		SkeletonPose(uint32 joints_size);
		SkeletonPose(const std::vector<JointTransform>& _local_transforms);

		bool is_valid() const;

		uint32 size() const;

		void set_joint_transform(const JointTransform& new_joint_transform, uint32 joint_id);
		void set_joint_translation(const vec3& new_translation, uint32 joint_id);
		void set_joint_scale(const vec3& new_scale, uint32 joint_id);
		void set_joint_rotation(const quat& new_rotation, uint32 joint_id);

		const JointTransform& get_joint_transform(uint32 joint_id) const;
		const vec3& get_joint_translation(uint32 joint_id) const;
		const vec3& get_joint_scale(uint32 joint_id) const;
		const quat& get_joint_rotation(uint32 joint_id) const;

	private:
		std::vector<JointTransform> local_transforms; // In parent space.

		friend class Skeleton;
		friend class SkeletonComponent;
	};

	class ERA_CORE_API Skeleton : public GameAsset
	{
	public:
		Skeleton() = default;

		void analyze_joints();

		void pretty_print_hierarchy() const;

		static std::string get_asset_type_impl();

		void set_joint_transform(const trs& new_transform, uint32 joint_id);
		void set_joint_rotation(const quat& new_rotation, uint32 joint_id);

		const trs& get_joint_transform(uint32 joint_id) const;
		const quat& get_joint_rotation(uint32 joint_id) const;

		void set_joint_translation(const vec3& new_translation, uint32 joint_id);
		const vec3& get_joint_translation(uint32 joint_id) const;

		uint32 get_joint_index(std::string_view name) const;

		const SkeletonPose& get_default_pose() const;

		bool serialize(std::ostream& os) const override;
		bool deserialize(std::istream& is) override;

	public:
		std::vector<SkeletonJoint> joints;
		std::vector<JointTransform> default_local_transforms; // In parent space.
		std::unordered_map<std::string, uint32> name_to_joint_id;

		uint32 root_joint_id = 0;

	private:
		mutable SkeletonPose default_pose = SkeletonPose();
	};

	class ERA_CORE_API SkeletonImportUtils
	{
	public:
		SkeletonImportUtils() = delete;

		static std::vector<ref<Skeleton>> import_skeletons(std::vector<SkeletonAssetImportData>& skeletons_to_import,
			const fs::path& file,
			uint32 flags = 0);
	};
}