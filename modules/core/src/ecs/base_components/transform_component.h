#pragma once

#include "core_api.h"

#include "core/math.h"

#include "ecs/component.h"

namespace era_engine
{

	class ERA_CORE_API TransformComponent final : public Component
	{
	public:
		enum TransformType : uint8_t
		{
			STATIC,
			DYNAMIC
		};

	public:
		TransformComponent() = default;
		TransformComponent(ref<Entity::EcsData> _data, const trs& t = trs::identity);
		TransformComponent(ref<Entity::EcsData> _data, const trs& t, TransformType _type);
		TransformComponent(ref<Entity::EcsData> _data, const vec3& position, const quat& rotation, const vec3& scale = vec3(1.f, 1.f, 1.f), TransformType _type = DYNAMIC);

		~TransformComponent() override;

		const trs& get_local_transform() const;
		void set_local_transform(const trs& new_local_transform);

		const trs& get_world_transform() const;
		void set_world_transform(const trs& new_world_transform);

		const vec3& get_world_position() const;
		void set_world_position(const vec3& new_world_position);

		const quat& get_world_rotation() const;
		void set_world_rotation(const quat& new_world_rotation);

		const vec3& get_world_scale() const;
		void set_world_scale(const vec3& new_world_scale);

		ERA_VIRTUAL_REFLECT(Component)

	private:
		void on_world_transform_requested() const;
		void on_world_transform_changed();

	public:
		TransformType type = DYNAMIC;

	private:
		trs local_transform = trs::identity; // In Parent space.
		mutable trs transform = trs::identity; // In World space.
		friend class EntityContainer;
	};

}