#include "motion_matching/features/trajectory_feature.h"
#include "motion_matching/trajectory/trajectory_component.h"
#include "motion_matching/motion/root_trajectory_utils.h"
#include "motion_matching/motion/motion_component.h"

#include <animation/animation_pose_sampler.h>
#include <animation/skeleton_component.h>
#include <animation/animation.h>

#include <ecs/base_components/transform_component.h>

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<TrajectoryFeatureDesc>("TrajectoryFeatureDesc")
			.constructor<>()(policy::ctor::as_raw_ptr);

		registration::class_<TrajectoryFeature>("TrajectoryFeature")
			.constructor<>()(policy::ctor::as_raw_ptr);
	}

	TrajectoryFeature::~TrajectoryFeature()
	{
	}

	uint32 TrajectoryFeature::get_feature_size() const
	{
		return uint32(descriptors.size()) * 2u;
	}

	std::vector<float> TrajectoryFeature::compute_features(const FeatureComputationContext& context) const
	{
		ASSERT(context.trajectory_component->number_of_trajectories == NUM_OF_TRAJECTORIES);

		std::vector<float> values;
		values.reserve(NUM_OF_TRAJECTORIES * descriptors.size());

		float current_time = 0.0f;

		const trs& world_transform = context.entity.get_component<TransformComponent>()->get_world_transform();

		for (int32 i = 0; i < NUM_OF_TRAJECTORIES; ++i)
		{
			for (ref<FeatureDesc> descriptor : descriptors)
			{
				if (ref<TrajectoryFeatureDesc> trajectory_feature_desc = std::dynamic_pointer_cast<TrajectoryFeatureDesc>(descriptor))
				{
					if (!fuzzy_equals(context.trajectory_component->time_offsets(i), trajectory_feature_desc->time_offset))
					{
						continue;
					}

					const trs trajectory_position = trs(context.trajectory_component->trajectory_positions(i), context.trajectory_component->trajectory_rotations(i), vec3(1.0f));

					const trs root_space_transform = invert(world_transform) * trajectory_position;

					if (trajectory_feature_desc->type == FeatureDescType::LOCATION)
					{
						const vec3& position = root_space_transform.position;

						values.emplace_back(position.x);
						values.emplace_back(position.z);
					}
					else if (trajectory_feature_desc->type == FeatureDescType::DIRECTION)
					{
						const quat& rotation = root_space_transform.rotation;
						const vec3 direction = noz(rotation * vec3::forward);

						values.emplace_back(direction.x);
						values.emplace_back(direction.z);
					}
					else if (trajectory_feature_desc->type == FeatureDescType::VELOCITY)
					{
						const vec3 velocity = conjugate(context.motion_component->get_input_movement_rotation()) * context.trajectory_component->trajectory_velocities(i);

						values.emplace_back(velocity.x);
						values.emplace_back(velocity.z);
					}
				}
				else
				{
					ASSERT(false);
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

		const uint32 num_samples = clip->get_num_samples_per_track();

		const float timestep = 1.0f / clip->get_sample_rate();
		float current_time = 0.0f;

		std::vector<float> descriptor_values_x;
		descriptor_values_x.reserve(num_samples);

		std::vector<float> descriptor_values_z;
		descriptor_values_z.reserve(num_samples);

		for (ref<FeatureDesc> descriptor : descriptors)
		{
			if (ref<TrajectoryFeatureDesc> trajectory_feature_desc = std::dynamic_pointer_cast<TrajectoryFeatureDesc>(descriptor))
			{
				for (uint32 i = 0; i < num_samples; ++i)
				{
					const trs current_transform = RootTrajectoryUtils::get_trs_from_origin(sampler, current_time, current_time + trajectory_feature_desc->time_offset);

					if (trajectory_feature_desc->type == FeatureDescType::LOCATION)
					{
						descriptor_values_x.emplace_back(current_transform.position.x);
						descriptor_values_z.emplace_back(current_transform.position.z);
					}
					else if (trajectory_feature_desc->type == FeatureDescType::DIRECTION)
					{
						const vec3 direction = noz(current_transform.rotation * vec3::forward);

						descriptor_values_x.emplace_back(direction.x);
						descriptor_values_z.emplace_back(direction.z);
					}
					else if (trajectory_feature_desc->type == FeatureDescType::VELOCITY)
					{
						const trs prev_transform = RootTrajectoryUtils::get_trs_from_origin(sampler, current_time, current_time + trajectory_feature_desc->time_offset - timestep);
						const vec3 velocity = (current_transform.position - prev_transform.position) / timestep;

						descriptor_values_x.emplace_back(velocity.x);
						descriptor_values_z.emplace_back(velocity.z);
					}
				}

				current_time += timestep;
			}
			else
			{
				ASSERT(false);
			}

			std::string x_curve_name = descriptor->name + "_x";
			clip->remove_curve(x_curve_name);
			clip->add_curve(x_curve_name, descriptor_values_x);

			std::string z_curve_name = descriptor->name + "_z";
			clip->remove_curve(z_curve_name);
			clip->add_curve(z_curve_name, descriptor_values_z);

			descriptor_values_x.clear();
			descriptor_values_z.clear();
		}

		return true;
	}

	bool TrajectoryFeature::sample_animation(const animation::Skeleton* skeleton, ref<animation::AnimationAssetClip> clip, float sample_rate, std::vector<ref<MotionMatchingDatabase::Sample>>& out_samples) const
	{
		using namespace animation;

		AnimationRootSampler sampler;
		sampler.init(skeleton, clip);

		const uint32 num_samples = std::lrintf(sample_rate * clip->get_duration());

		const float timestep = 1.0f / sample_rate;
		float current_time = 0.0f;

		for (uint32 i = 0; i < num_samples; ++i)
		{
			ref<MotionMatchingDatabase::Sample>& sample = out_samples[i];

			for (ref<FeatureDesc> descriptor : descriptors)
			{
				if (ref<TrajectoryFeatureDesc> trajectory_feature_desc = std::dynamic_pointer_cast<TrajectoryFeatureDesc>(descriptor))
				{
					float x_value = 0.0f;
					std::string x_curve_name = descriptor->name + "_x";
					clip->sample_curve(current_time, x_curve_name, x_value);

					float z_value = 0.0f;
					std::string z_curve_name = descriptor->name + "_z";
					clip->sample_curve(current_time, z_curve_name, z_value);

					sample->features.emplace_back(x_value);
					sample->features.emplace_back(z_value);
				}
				else
				{
					ASSERT(false);
				}
			}

			current_time += timestep;
		}

		return true;
	}
}