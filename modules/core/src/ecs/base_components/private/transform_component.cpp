#include "ecs/base_components/transform_component.h"
#include "ecs/world.h"

#include <rttr/registration>

namespace era_engine
{

	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::enumeration<TransformComponent::TransformType>("TransformType")
			(
				value("STATIC", TransformComponent::STATIC),
				value("DYNAMIC", TransformComponent::DYNAMIC)
			);

		registration::class_<TransformComponent>("TransformComponent")
			.constructor<ref<Entity::EcsData>, const trs&>()
			.constructor<ref<Entity::EcsData>, const trs&, TransformComponent::TransformType>()
			.constructor<ref<Entity::EcsData>, const vec3&, const quat&, const vec3&, TransformComponent::TransformType>()
			.property("transform", &TransformComponent::get_world_transform, &TransformComponent::set_world_transform)
			.property("local_transform", &TransformComponent::get_local_transform, &TransformComponent::set_local_transform)
			.property("type", &TransformComponent::type);
	}

	TransformComponent::TransformComponent(ref<Entity::EcsData> _data, const trs& t)
		: Component(_data), transform(t)
	{
	}

	TransformComponent::TransformComponent(ref<Entity::EcsData> _data, const trs& t, TransformType _type)
		: Component(_data), transform(t), type(_type)
	{
	}

	TransformComponent::TransformComponent(ref<Entity::EcsData> _data, const vec3& position, const quat& rotation, const vec3& scale, TransformType _type)
		: Component(_data), transform(position, rotation, scale), type(_type)
	{
	}

	TransformComponent::~TransformComponent()
	{
	}

	const trs& TransformComponent::get_local_transform() const
	{
		return local_transform;
	}

	void TransformComponent::set_local_transform(const trs& new_local_transform)
	{
		local_transform = new_local_transform;
	}

	const trs& TransformComponent::get_world_transform() const
	{
		on_world_transform_requested();

		return transform;
	}

	void TransformComponent::set_world_transform(const trs& new_world_transform)
	{
		transform = new_world_transform;

		on_world_transform_changed();
	}

	const vec3& TransformComponent::get_world_position() const
	{
		on_world_transform_requested();

		return transform.position;
	}

	void TransformComponent::set_world_position(const vec3& new_world_position)
	{
		transform.position = new_world_position;

		on_world_transform_changed();
	}

	const quat& TransformComponent::get_world_rotation() const
	{
		on_world_transform_requested();

		return transform.rotation;
	}

	void TransformComponent::set_world_rotation(const quat& new_world_rotation)
	{
		transform.rotation = new_world_rotation;

		on_world_transform_changed();
	}

	void TransformComponent::on_world_transform_changed()
	{
		const Entity::Handle parent_handle = get_entity().get_parent_handle();

		if (parent_handle != Entity::NullHandle)
		{
			TransformComponent* parent_transform = get_world()->get_entity(parent_handle).get_component<TransformComponent>();
			local_transform = invert(parent_transform->get_world_transform()) * transform;
		}
		else
		{
			local_transform = transform;
		}
	}

	void TransformComponent::on_world_transform_requested() const
	{
		std::vector<const TransformComponent*> transforms_to_consider;
		transforms_to_consider.emplace_back(this);

		World* world = get_world();

		Entity root_entity = world->get_root_entity();

		Entity::Handle ancestor = get_entity().get_parent_handle();
		while (ancestor != Entity::NullHandle && ancestor != root_entity.get_handle())
		{
			Entity ancestor_entity = world->get_entity(ancestor);
			TransformComponent* ancestor_transform_component = ancestor_entity.get_component<TransformComponent>();
			if (ancestor_transform_component != nullptr)
			{
				transforms_to_consider.emplace_back(ancestor_transform_component);
			}

			ancestor = ancestor_entity.get_parent_handle();
		}

		trs transform_accumulator = trs::identity;

		for (auto iter = transforms_to_consider.rbegin(); iter != transforms_to_consider.rend(); ++iter)
		{
			const TransformComponent* current_transform_component = *iter;

			transform_accumulator = transform_accumulator * current_transform_component->local_transform;

			current_transform_component->transform = transform_accumulator;
		}
	}
}