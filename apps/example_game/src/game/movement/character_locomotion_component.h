#pragma once

#include <ecs/component.h>

#include <core/math.h>

namespace era_engine
{
	class SearchResult;

	class CharacterLocomotionComponent final : public Component
	{
	public:
		CharacterLocomotionComponent() = default;
		CharacterLocomotionComponent(ref<Entity::EcsData> _data);

		~CharacterLocomotionComponent() override;

		std::string get_motion_database_id() const;
		bool need_force_start_search() const;
		void on_search_succeeded(const SearchResult&) const;

		mutable vec3 prev_input;

		ERA_VIRTUAL_REFLECT(Component)
	};
}