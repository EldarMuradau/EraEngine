#include "motion_matching/features/trajectory_feature.h"

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
		registration::class_<TrajectoryFeature>("TrajectoryFeature")
			.constructor<>()(policy::ctor::as_raw_ptr);
	}

	TrajectoryFeature::~TrajectoryFeature()
	{
	}

	void TrajectoryFeature::compute_features(const FeatureComputationContext& context)
	{
	}

	bool TrajectoryFeature::mark_animation(Entity entity, ref<animation::AnimationAssetClip> clip) const
	{
		using namespace animation;

		SkeletonComponent* skeleton_component = entity.get_component<SkeletonComponent>();

		AnimationRootSampler sampler;
		sampler.init(skeleton_component->skeleton.get(), clip);

		const uint32 samples_per_track = clip->get_num_samples_per_track();
		const float sample_time = 1.0f / clip->get_sample_rate();

		std::vector<float> descriptor_values_x;
		descriptor_values_x.reserve(samples_per_track);

		std::vector<float> descriptor_values_z;
		descriptor_values_z.reserve(samples_per_track);

		for (ref<FeatureDesc> descriptor : descriptors)
		{
			JointTransform prev_root_pose = sampler.sample_root(0.0f);

			for (uint32 i = 0; i < samples_per_track; ++i)
			{
				const float current_time = sample_time * static_cast<float>(i);

				JointTransform current_root_pose = sampler.sample_root(current_time);

				if (descriptor->type == FeatureDesc::Type::LOCATION)
				{
					descriptor_values_x.emplace_back(current_root_pose.get_translation().x);
					descriptor_values_z.emplace_back(current_root_pose.get_translation().z);
				}
				else if (descriptor->type == FeatureDesc::Type::DIRECTION)
				{
					const vec3 joint_direction = noz(current_root_pose.get_translation());

					descriptor_values_x.emplace_back(joint_direction.x);
					descriptor_values_z.emplace_back(joint_direction.z);
				}
				else if (descriptor->type == FeatureDesc::Type::VELOCITY)
				{
					const vec3 joint_velocity = (current_root_pose.get_translation() - prev_root_pose.get_translation()) / sample_time;

					descriptor_values_x.emplace_back(joint_velocity.x);
					descriptor_values_z.emplace_back(joint_velocity.z);
				}

				prev_root_pose = current_root_pose;
			}

			std::string x_curve_name = descriptor->name + "_x";
			clip->add_curve(x_curve_name, descriptor_values_x);


			std::string z_curve_name = descriptor->name + "_z";
			clip->add_curve(z_curve_name, descriptor_values_z);

			descriptor_values_x.clear();
			descriptor_values_z.clear();
		}

		return true;
	}
}