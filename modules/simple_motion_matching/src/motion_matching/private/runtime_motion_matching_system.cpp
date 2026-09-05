#include "motion_matching/runtime_motion_matching_system.h"
#include "motion_matching/features/motion_matching_feature_set.h"
#include "motion_matching/motion_matching_database.h"
#include "motion_matching/common.h"
#include "motion_matching/motion_data_component.h"
#include "motion_matching/motion_matching_component.h"
#include "motion_matching/features/motion_matching_feature.h"

#include "animation/animation.h"

#include "core/string.h"
#include "core/debug/debug_var.h"
#include "core/cpu_profiling.h"

#include "ecs/update_groups.h"
#include "ecs/base_components/transform_component.h"

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<RuntimeMotionMatchingSystem>("RuntimeMotionMatchingSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("motion_matching")))
			.method("update", &RuntimeMotionMatchingSystem::update)(metadata("update_group", update_types::GAMEPLAY_BEFORE_PHYSICS),
				metadata("Before", std::vector<std::string>{"MotionSystem::update"}));
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

		ZoneScopedN("RuntimeMotionMatchingSystem::update");

        for (auto&& [handle, transform_component, mm_controller, motion_data_component, animation_component] : world->group(components_group<TransformComponent, 
			MotionMatchingComponent, 
			MotionDataComponent, 
			AnimationComponent>).each())
        {
			motion_data_component.search_timer -= dt;

			if (motion_data_component.on_update_func != nullptr)
			{
				motion_data_component.on_update_func(dt);
			}

			const bool is_motion_recuperation_time = motion_data_component.search_timer <= 0.0f;
			const bool need_to_force_trigger_search = motion_data_component.need_force_start_search_func != nullptr ? motion_data_component.need_force_start_search_func() : true;

			if (is_motion_recuperation_time || need_to_force_trigger_search)
			{
				if (motion_data_component.on_search_time_func != nullptr)
				{
					motion_data_component.on_search_time_func();
				}

				if(is_motion_recuperation_time && motion_data_component.on_search_recuperation_func != nullptr)
				{
					motion_data_component.on_search_recuperation_func();
				}

				Entity entity = world->get_entity(handle);

				FeatureComputationContext context;
				context.fill_context(entity, dt);

				const std::string database_id = motion_data_component.get_motion_database_id_func();
				if (database_id.empty())
				{
					if (motion_data_component.on_search_failed_func != nullptr)
					{
						motion_data_component.on_search_failed_func();
					}
					continue;
				}

				ref<MotionMatchingDatabase> database = MotionDatabaseRegistry::get_by_id(database_id);

				MotionMatchingFeatureSet feature_set;
				if (motion_data_component.calculate_feature_set_func != nullptr)
				{
					feature_set = motion_data_component.calculate_feature_set_func(context);
				}
				else
				{
					for (const MotionMatchingFeature* feature : database->features)
					{
						std::vector<float> feature_vector = feature->compute_features(context);
						feature_set.add_feature(feature->get_type(), std::move(feature_vector));
					}
				}

				SearchResult search_result = mm_controller.search_animation(feature_set, database.get());
				motion_data_component.search_timer = motion_data_component.search_time;
				
				if (search_result.animation != nullptr)
				{
					if (motion_data_component.on_search_succeeded_func != nullptr)
					{
						motion_data_component.on_search_succeeded_func(search_result);
					}

					animation_component.current_animation = search_result.animation;
					animation_component.current_anim_position = search_result.anim_position;
				}
				else
				{
					if (motion_data_component.on_search_failed_func != nullptr)
					{
						motion_data_component.on_search_failed_func();
					}
				}
			}
        }
	}
}