#include "motion_matching/runtime_motion_matching_system.h"
#include "motion_matching/features/motion_matching_feature_set.h"
#include "motion_matching/motion_matching_database.h"
#include "motion_matching/common.h"
#include "motion_matching/motion_data_component.h"
#include "motion_matching/motion_matching_component.h"

#include "animation/animation.h"

#include "core/string.h"
#include "core/debug/debug_var.h"

#include "ecs/update_groups.h"
#include "ecs/base_components/transform_component.h"

#include <rttr/policy.h>
#include <rttr/registration>
#include <motion_matching/features/motion_matching_feature.h>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<RuntimeMotionMatchingSystem>("RuntimeMotionMatchingSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("motion_matching")))
			.method("update", &RuntimeMotionMatchingSystem::update)(metadata("update_group", update_types::GAMEPLAY_BEFORE_PHYSICS));
	}

	RuntimeMotionMatchingSystem::RuntimeMotionMatchingSystem(World* _world)
		: System(_world)
	{
	}

	RuntimeMotionMatchingSystem::~RuntimeMotionMatchingSystem()
	{
	}

	void RuntimeMotionMatchingSystem::init()
	{
	}

	void RuntimeMotionMatchingSystem::update(float dt)
	{
		using namespace animation;

        for (auto&& [handle, transform_component, mm_controller, motion_data_component, animation_component] : world->group(components_group<TransformComponent, 
			MotionMatchingComponent, 
			MotionDataComponent, 
			AnimationComponent>).each())
        {
			motion_data_component.search_timer -= dt;

			motion_data_component.on_update_func(dt);

			const bool is_motion_recuperation_time = motion_data_component.search_timer <= 0.0f;
			const bool need_to_force_trigger_search = motion_data_component.need_force_start_search_func();

			if (is_motion_recuperation_time || need_to_force_trigger_search)
			{
				motion_data_component.on_search_time_func();

				if(is_motion_recuperation_time)
				{
					motion_data_component.on_search_recuperation_func();
				}

				Entity entity = world->get_entity(handle);

				FeatureComputationContext context;
				context.fill_context(entity, dt);

				MotionMatchingFeatureSet feature_set = motion_data_component.calculate_feature_set_func(context);

				SearchResult search_result = mm_controller.search_animation(feature_set, motion_data_component.get_motion_database_id_func());
				motion_data_component.search_timer = motion_data_component.search_time;
				
				if (search_result.animation != nullptr)
				{
					animation_component.current_animation = search_result.animation;
					animation_component.current_anim_position = search_result.anim_position;

					motion_data_component.on_search_succeeded_func(search_result);
				}
				else
				{
					motion_data_component.on_search_failed_func();
				}
			}
        }
	}
}