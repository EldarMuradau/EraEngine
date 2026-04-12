#include "motion_matching/features/trajectory_feature.h"
#include "motion_matching/trajectory/trajectory_component.h"

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

	std::vector<float> TrajectoryFeature::compute_features(const FeatureComputationContext& context)
	{
		std::vector<float> values;
		values.reserve(context.trajectory_component->number_of_trajectories * descriptors.size());

		for (int32 i = 0; i < context.trajectory_component->number_of_trajectories; ++i)
		{
			for (ref<FeatureDesc> descriptor : descriptors)
			{
				if (descriptor->type == FeatureDesc::Type::LOCATION)
				{
					const vec3& position = context.trajectory_component->trajectory_positions(i);

					values.emplace_back(position.x);
					values.emplace_back(position.z);
				}
				else if (descriptor->type == FeatureDesc::Type::DIRECTION)
				{
					const quat& rotation = context.trajectory_component->trajectory_rotations(i);
					const vec3 direction = noz(rotation * vec3::forward);

					values.emplace_back(direction.x);
					values.emplace_back(direction.z);
				}
				else if (descriptor->type == FeatureDesc::Type::VELOCITY)
				{
					const vec3& velocity = context.trajectory_component->trajectory_velocities(i);

					values.emplace_back(velocity.x);
					values.emplace_back(velocity.z);
				}
			}
		}

		return values;
	}

	bool TrajectoryFeature::mark_animation(const animation::SkeletonComponent* skeleton_component, ref<animation::AnimationAssetClip> clip) const
	{
		using namespace animation;

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
					const vec3 joint_direction = noz(current_root_pose.get_rotation() * vec3::forward);

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

	bool TrajectoryFeature::sample_animation(ref<animation::AnimationAssetClip> clip, float sample_rate, std::vector<ref<MotionMatchingDatabase::Sample>>& out_samples) const
	{
		const uint32 num_samples = std::lrintf(sample_rate * clip->get_duration());

		const float timestep = 1.0f / sample_rate;
		float current_time = 0.0f;

		for (uint32 i = 0; i < num_samples; ++i)
		{
			ref<MotionMatchingDatabase::Sample>& sample = out_samples[i];

			for (ref<FeatureDesc> descriptor : descriptors)
			{
				std::string x_curve_name = descriptor->name + "_x";
				float out_x = 0.0f;
				clip->sample_curve(current_time, x_curve_name, out_x);
				sample->features.emplace_back(out_x);

				std::string z_curve_name = descriptor->name + "_z";
				float out_z = 0.0f;
				clip->sample_curve(current_time, z_curve_name, out_z);
				sample->features.emplace_back(out_z);
			}

			current_time += timestep;
		}

		return true;
	}
}