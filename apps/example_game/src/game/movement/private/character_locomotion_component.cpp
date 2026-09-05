#include "game/movement/character_locomotion_component.h"

#include <ecs/entity.h>
#include <ecs/base_components/transform_component.h>

#include <core/ecs/input_receiver_component.h>

#include <motion_matching/motion_data_component.h>
#include <motion_matching/motion/motion_component.h>

#include <animation/animation_pose_sampler.h>
#include <animation/animation.h>

#include <rttr/registration>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<CharacterLocomotionComponent>("CharacterLocomotionComponent")
			.constructor<ref<Entity::EcsData>>();
	}

	CharacterLocomotionComponent::CharacterLocomotionComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
		Entity entity = get_entity();

		MotionDataComponent* motion_data_component = entity.get_component<MotionDataComponent>();

		motion_data_component->get_motion_database_id_func = std::bind(&CharacterLocomotionComponent::get_motion_database_id, this);
		motion_data_component->on_search_succeeded_func = std::bind(&CharacterLocomotionComponent::on_search_succeeded, this, std::placeholders::_1);
		motion_data_component->need_force_start_search_func = std::bind(&CharacterLocomotionComponent::need_force_start_search, this);

		motion_data_component->search_time = 0.28f;
	}

	CharacterLocomotionComponent::~CharacterLocomotionComponent()
	{
	}

	std::string CharacterLocomotionComponent::get_motion_database_id() const
	{
		return std::string("LOCOMOTION");
	}

	bool CharacterLocomotionComponent::need_force_start_search() const
	{
		const MotionComponent* motion_component = get_entity().get_component<MotionComponent>();

		const vec3& current_input = motion_component->get_current_input();

		if (!fuzzy_equals(current_input, prev_input))
		{
			prev_input = current_input;
			return true;
		}

		return false;
	}

	void CharacterLocomotionComponent::on_search_succeeded(const SearchResult& result) const
	{
		using namespace animation;
		Entity entity = get_entity();

		AnimationComponent* animation_component = entity.get_component<AnimationComponent>();
		const SkeletonComponent* skeleton_component = entity.get_component<SkeletonComponent>();

		const Skeleton* skeleton = skeleton_component->skeleton.get();

		AnimationPoseSampler sampler;
		sampler.init(skeleton, result.animation);

		if (!animation_component->enable_root_motion)
		{
			sampler.set_joint_enabled(skeleton->joints[0].name, false);
		}

		SkeletonPose result_pose = SkeletonPose(skeleton->joints.size());
		sampler.sample_pose(result.anim_position, result_pose);

		animation_component->trigger_reset_inertial_blend(result_pose, 0.18f);

		//MotionComponent* motion_component = entity.get_component<MotionComponent>();
		//motion_component->init_root_motion(skeleton, 
		//	result.animation, 
		//	result.anim_position, 
		//	0.28f,
		//	RootMotionType::LOCATION);
	}

}