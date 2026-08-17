#include "physics/destructions/destructible_component.h"

#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<DestructibleComponent>("DestructibleComponent")
			.constructor<>();

		registration::class_<DestructibleFractureChunkComponent>("DestructibleFractureChunkComponent")
			.constructor<>();
	}

	DestructibleComponent::DestructibleComponent()
	{
	}

	DestructibleComponent::DestructibleComponent(ref<Entity::EcsData> _data, Type _base_type)
		: Component(_data)
		, base_type(_base_type)
	{
		if (base_type == Type::FRACTURE_BASED)
		{
			fracture_desc = FractureDescriptor();
		}
		else if (base_type == Type::FRACTURE_BASED)
		{
			family_asset = nullptr;
		}
	}

	DestructibleComponent::~DestructibleComponent()
	{
	}

	DestructibleComponent& DestructibleComponent::operator=(const DestructibleComponent& other)
	{
		Component::operator=(other);

		base_type = other.base_type;
		load_state = other.load_state;

		if (base_type == Type::FAMILY_BASED)
		{
			family_asset = other.family_asset;
		}
		else
		{
			fracture_desc = other.fracture_desc;
		}

		chunks = other.chunks;

		return *this;
	}

	DestructibleComponent& DestructibleComponent::operator=(DestructibleComponent&& other)
	{
		if (*this == other)
		{
			return *this;
		}

		Component::operator=(std::forward<DestructibleComponent>(other));

		base_type = other.base_type;
		load_state = other.load_state;

		if (base_type == Type::FAMILY_BASED)
		{
			family_asset = std::move(other.family_asset);
		}
		else
		{
			fracture_desc = std::move(other.fracture_desc);
		}

		chunks = std::move(other.chunks);

		return *this;
	}

	bool DestructibleComponent::operator==(const DestructibleComponent& other) const
	{
		return get_handle() == other.get_handle();
	}

	bool DestructibleComponent::operator!=(const DestructibleComponent& other) const
	{
		return !operator==(other);
	}

	DestructibleFractureChunkComponent::DestructibleFractureChunkComponent()
	{
	}

	DestructibleFractureChunkComponent::DestructibleFractureChunkComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
	}

	DestructibleFractureChunkComponent::~DestructibleFractureChunkComponent()
	{
	}
}