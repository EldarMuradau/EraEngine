// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "ai/navigation_component.h"
#include "ai/navigation.h"

#include "ecs/base_components/transform_component.h"
#include "ecs/world.h"

#include <rttr/registration>

namespace era_engine::ai
{
	NODISCARD static CoroutineReturn<NavNode> navigate(const vec2& pos, const vec2& target)
	{
		NavNode player;
		player.position = pos / (float)NAV_X_STEP;

		NavNode destination;
		destination.position = target / (float)NAV_X_STEP;

		const auto& path = AStarNavigation::navigate(player, destination);
		for (const NavNode& node : path)
		{
			co_yield node;
		}

		while (true)
		{
			co_yield NavNode(vec2(NAV_INF_POS));
		}

		co_return NavNode(vec2(NAV_INF_POS));
	}

	static bool equal_in_2d(const vec3& lhs, const vec3& rhs)
	{
		return (int)rhs.x == (int)lhs.x && (int)rhs.z == (int)lhs.z;
	}

	RTTR_REGISTRATION
	{
		using namespace rttr;
		rttr::registration::class_<NavigationComponent>("NavigationComponent")
			.constructor<>()
			.constructor<ref<Entity::EcsData>, NavigationComponent::NavType>()
			.property("destination", &NavigationComponent::destination)
			.property("type", &NavigationComponent::type);
	}

	void NavigationComponent::process_path()
	{
		TransformComponent* transform = get_world()->get_entity(component_data->entity_handle).get_component<TransformComponent>();
		const auto& pos = transform->get_world_position();

		if (!equal_in_2d(destination, previous_destination))
		{
			create_path(destination, pos);
			previous_destination = destination;
		}

		Coroutine<NavNode> nav_cor = nav_coroutine;
		if (nav_cor)
		{
			NavNode temp_pos = nav_cor.value();
			if (temp_pos.position != vec2(NAV_INF_POS))
			{
				transform->set_world_position(lerp(pos, vec3(temp_pos.position.x, 0, temp_pos.position.y), 0.025f));

				if (length(transform->get_world_position() - vec3(temp_pos.position.x, 0, temp_pos.position.y)) < 0.25f)
				{
					nav_cor();
				}
			}
			else
			{
				nav_cor.token->cancelled = true;
				nav_cor.destroy();
			}
		}
	}

	void NavigationComponent::create_path(const vec3& to, const vec3& from)
	{
		nav_coroutine.handle = {};
		nav_coroutine = navigate(vec2((unsigned int)from.x, (unsigned int)from.z), vec2((unsigned int)to.x, (unsigned int)to.z));
	}

	NavigationComponent::NavigationComponent(ref<Entity::EcsData> _data, NavType _type)
		: Component(_data), type(_type)
	{
	}

	NavigationComponent::~NavigationComponent()
	{
	}

}