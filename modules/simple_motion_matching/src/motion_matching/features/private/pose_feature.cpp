#include "motion_matching/features/pose_feature.h"

#include <animation/animation_pose_sampler.h>
#include <animation/skeleton_component.h>
#include <animation/animation.h>

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<PoseFeature>("PoseFeature")
			.constructor<>()(policy::ctor::as_raw_ptr);
	}

	PoseFeature::~PoseFeature()
	{
	}

	void PoseFeature::compute_features(const FeatureComputationContext& context)
	{
		using namespace animation;

		ref<AnimationAssetClip> animation = context.animation_component->current_animation;

		const float current_time = context.animation_component->current_anim_position;

		values.clear();

		for (ref<FeatureDesc> descriptor : descriptors)
		{
			if (ref<PoseFeatureDesc> pose_feature_desc = std::dynamic_pointer_cast<PoseFeatureDesc>(descriptor))
			{
				if (pose_feature_desc->basis == FeatureDesc::Basis::XYZ)
				{
					float x_value = 0.0f;
					std::string x_curve_name = pose_feature_desc->name + "_x";
					animation->sample_curve(current_time, x_curve_name, x_value);

					float y_value = 0.0f;
					std::string y_curve_name = pose_feature_desc->name + "_y";
					animation->sample_curve(current_time, y_curve_name, y_value);

					float z_value = 0.0f;
					std::string z_curve_name = pose_feature_desc->name + "_z";
					animation->sample_curve(current_time, z_curve_name, z_value);

					values.emplace_back(x_value);
					values.emplace_back(y_value);
					values.emplace_back(z_value);
				}
				else if (pose_feature_desc->basis == FeatureDesc::Basis::XZ)
				{
					float x_value = 0.0f;
					std::string x_curve_name = pose_feature_desc->name + "_x";
					animation->sample_curve(current_time, x_curve_name, x_value);

					float z_value = 0.0f;
					std::string z_curve_name = pose_feature_desc->name + "_z";
					animation->sample_curve(current_time, z_curve_name, z_value);

					values.emplace_back(x_value);
					values.emplace_back(z_value);
				}
				else if (pose_feature_desc->basis == FeatureDesc::Basis::Y)
				{
					float y_value = 0.0f;
					std::string y_curve_name = pose_feature_desc->name + "_y";
					animation->sample_curve(current_time, y_curve_name, y_value);

					values.emplace_back(y_value);
				}
			}
			else
			{
				ASSERT(false);
			}
		}
	}

	bool PoseFeature::mark_animation(Entity entity, ref<animation::AnimationAssetClip> clip) const
	{
		using namespace animation;

		SkeletonComponent* skeleton_component = entity.get_component<SkeletonComponent>();

		AnimationPoseSampler sampler;
		sampler.init(skeleton_component->skeleton.get(), clip);

		const uint32 samples_per_track = clip->get_num_samples_per_track();
		const float sample_time = 1.0f / clip->get_sample_rate();

		std::vector<float> descriptor_values_x;
		descriptor_values_x.reserve(samples_per_track);

		std::vector<float> descriptor_values_y;
		descriptor_values_y.reserve(samples_per_track);

		std::vector<float> descriptor_values_z;
		descriptor_values_z.reserve(samples_per_track);

		for (ref<FeatureDesc> descriptor : descriptors)
		{
			if (ref<PoseFeatureDesc> pose_feature_desc = std::dynamic_pointer_cast<PoseFeatureDesc>(descriptor))
			{
				SkeletonPose prev_pose = skeleton_component->skeleton->get_default_pose();
				sampler.sample_pose(0.0f, prev_pose);

				for (uint32 i = 0; i < samples_per_track; ++i)
				{
					const float current_time = sample_time * static_cast<float>(i);

					SkeletonPose current_pose = skeleton_component->skeleton->get_default_pose();
					sampler.sample_pose(current_time, current_pose);

					const trs current_joint_transform = SkeletonUtils::get_object_space_joint_transform(current_pose, skeleton_component->skeleton.get(), pose_feature_desc->joint_id);

					if (pose_feature_desc->type == FeatureDesc::Type::LOCATION)
					{
						descriptor_values_x.emplace_back(current_joint_transform.position.x);
						descriptor_values_y.emplace_back(current_joint_transform.position.y);
						descriptor_values_z.emplace_back(current_joint_transform.position.z);
					}
					else if (pose_feature_desc->type == FeatureDesc::Type::DIRECTION)
					{
						const vec3 joint_direction = noz(current_joint_transform.position);

						descriptor_values_x.emplace_back(joint_direction.x);
						descriptor_values_y.emplace_back(joint_direction.y);
						descriptor_values_z.emplace_back(joint_direction.z);
					}
					else if (pose_feature_desc->type == FeatureDesc::Type::VELOCITY)
					{
						const trs prev_joint_transform = SkeletonUtils::get_object_space_joint_transform(prev_pose, skeleton_component->skeleton.get(), pose_feature_desc->joint_id);

						const vec3 joint_velocity = (current_joint_transform.position - prev_joint_transform.position) / sample_time;

						descriptor_values_x.emplace_back(joint_velocity.x);
						descriptor_values_y.emplace_back(joint_velocity.y);
						descriptor_values_z.emplace_back(joint_velocity.z);
					}

					prev_pose = current_pose;
				}

				std::string x_curve_name = pose_feature_desc->name + "_x";
				clip->add_curve(x_curve_name, descriptor_values_x);

				std::string y_curve_name = pose_feature_desc->name + "_y";
				clip->add_curve(y_curve_name, descriptor_values_y);

				std::string z_curve_name = pose_feature_desc->name + "_z";
				clip->add_curve(z_curve_name, descriptor_values_z);

				descriptor_values_x.clear();
				descriptor_values_y.clear();
				descriptor_values_z.clear();
			}
			else
			{
				ASSERT(false);
				return false;
			}
		}

		return true;
	}
}