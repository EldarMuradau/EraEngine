#pragma once

#include "physics_api.h"

#include "physics/destructions/destruction_types.h"

#include <core/math.h>

#include <ecs/component.h>

namespace era_engine::physics
{
    enum class AnchorFlags : uint8
    {
        NONE = 0,
        LEFT = 1 << 0,
        RIGHT = 1 << 1,
        BOTTOM = 1 << 2,
        TOP = 1 << 3,
        FRONT = 1 << 4,
        BACK = 1 << 5
    };

	DEFINE_BITWISE_OPERATORS_FOR_ENUM(AnchorFlags);

	struct alignas(16) FractureDescriptor
	{
		uint32 chunks_count = 5;
		float break_force = 100.0f;

		AnchorFlags anchor = AnchorFlags::NONE;
	};

	class ERA_PHYSICS_API DestructibleComponent : public Component
	{
	public:
		enum class Type : uint8
		{
			FRACTURE_BASED = 0,
			FAMILY_BASED
		};

		DestructibleComponent();
		DestructibleComponent(ref<Entity::EcsData> _data, Type _base_type);
		~DestructibleComponent() override;

		Type base_type = Type::FRACTURE_BASED;

		union
		{
			ref<DestructibleAsset> family_asset; // Only if FAMILY_BASED.
			FractureDescriptor fracture_desc; // Only if FRACTURE_BASED.
		};

	protected:
		friend class DestructionSystem;
	};
}