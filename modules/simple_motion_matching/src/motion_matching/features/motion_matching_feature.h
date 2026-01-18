#pragma once

#include "motion_matching_api.h"

#include <animation/animation_clip.h>

#include <ecs/reflection.h>
#include <ecs/entity.h>

namespace era_engine
{
	namespace animation
	{
		class SkeletonComponent;
		class AnimationComponent;
	}

	class TrajectoryComponent;
	class MotionComponent;

	struct ERA_MOTION_MATCHING_API FeatureComputationContext
	{
		void fill_context(Entity _entity, float _dt);

		World* world = nullptr;

		Entity entity = Entity::Null;

		animation::SkeletonComponent* skeleton_component = nullptr;
		animation::AnimationComponent* animation_component = nullptr;

		TrajectoryComponent* trajectory_component = nullptr;
		MotionComponent* motion_component = nullptr;

		float dt = 0.0f;
	};

	class ERA_MOTION_MATCHING_API FeatureDesc
	{
	public:
		FeatureDesc() = default;
		virtual ~FeatureDesc() = default;

		enum class Type : uint8
		{
			LOCATION = 0,
			VELOCITY,
			DIRECTION
		};

		enum class Basis : uint8
		{
			XYZ = 0,
			XZ,
			Y
		};

		Type type = Type::LOCATION;
		Basis basis = Basis::XYZ;

		std::string name;
	};

	class ERA_MOTION_MATCHING_API MotionMatchingFeature
	{
	public:
		MotionMatchingFeature() = default;
		MotionMatchingFeature(const MotionMatchingFeature&) = default;
		virtual ~MotionMatchingFeature();

		std::vector<float> get_values() const;

		const std::vector<ref<FeatureDesc>>& get_descriptors() const;

		virtual void compute_features(const FeatureComputationContext& context);
		void store_features(std::vector<float>&& _values);

		virtual bool mark_animation(Entity entity, ref<animation::AnimationAssetClip> clip) const;

		ERA_REFLECT

	protected:
		std::vector<float> values;
		std::vector<ref<FeatureDesc>> descriptors;
	};
}