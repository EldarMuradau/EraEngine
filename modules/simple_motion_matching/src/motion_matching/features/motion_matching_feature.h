#pragma once

#include "motion_matching_api.h"
#include "motion_matching/motion_matching_database.h"

#include <animation/animation_clip.h>

#include <core/serialization/binary_serializer.h>

#include <ecs/reflection.h>
#include <ecs/entity.h>

namespace era_engine
{
	namespace animation
	{
		class SkeletonComponent;
		class Skeleton;
		class AnimationComponent;
	}

	class TrajectoryComponent;
	class MotionComponent;

	struct ERA_MOTION_MATCHING_API FeatureComputationContext
	{
		void fill_context(Entity _entity, float _dt);

		World* world = nullptr;

		Entity entity = Entity::Null;

		const animation::SkeletonComponent* skeleton_component = nullptr;
		const animation::AnimationComponent* animation_component = nullptr;

		const TrajectoryComponent* trajectory_component = nullptr;
		const MotionComponent* motion_component = nullptr;

		float dt = 0.0f;
	};

	enum class FeatureDescType : uint8
	{
		LOCATION = 0,
		VELOCITY,
		DIRECTION
	};

	enum class FeatureDescBasis : uint8
	{
		Y = 1,
		XZ = 2,
		XYZ = 3,
	};

	class ERA_MOTION_MATCHING_API FeatureDesc
	{
	public:
		FeatureDesc() = default;
		virtual ~FeatureDesc() = default;

		std::string name;

		ERA_BASE_VIRTUAL_BINARY_SERIALIZE(name);

		ERA_REFLECT
	};

	class ERA_MOTION_MATCHING_API MotionMatchingFeature
	{
	public:
		MotionMatchingFeature() = default;
		MotionMatchingFeature(const MotionMatchingFeature&) = default;
		virtual ~MotionMatchingFeature();

		const std::vector<ref<FeatureDesc>>& get_descriptors() const;

		virtual uint32 get_feature_size() const;

		virtual std::vector<float> compute_features(const FeatureComputationContext& context) const;

		virtual bool mark_animation(const animation::SkeletonComponent* skeleton_component, ref<animation::AnimationAssetClip> clip) const;

		virtual bool sample_animation(const animation::Skeleton* skeleton, ref<animation::AnimationAssetClip> clip, float sample_rate, std::vector<ref<MotionMatchingDatabase::Sample>>& out_samples) const;

		std::vector<ref<FeatureDesc>> descriptors;

		ERA_REFLECT
	};
}