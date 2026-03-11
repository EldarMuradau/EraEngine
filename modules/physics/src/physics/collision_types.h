#pragma once

#include "physics_api.h"

namespace era_engine::physics
{
	enum class CollisionType : uint32
	{
		NONE = 0,
		COLLISION_0 = uint32(1u << 0u),
		COLLISION_1 = uint32(1u << 1u),
		COLLISION_2 = uint32(1u << 2u),
		COLLISION_3 = uint32(1u << 3u),
		COLLISION_4 = uint32(1u << 4u),
		COLLISION_5 = uint32(1u << 5u),
		COLLISION_6 = uint32(1u << 6u),
		COLLISION_7 = uint32(1u << 7u),
		COLLISION_8 = uint32(1u << 8u),
		COLLISION_9 = uint32(1u << 9u),
		COLLISION_10 = uint32(1u << 10u),
		COLLISION_11 = uint32(1u << 11u),
		COLLISION_12 = uint32(1u << 12u),
		COLLISION_13 = uint32(1u << 13u),
		COLLISION_14 = uint32(1u << 14u),
		COLLISION_15 = uint32(1u << 15u),
		COLLISION_16 = uint32(1u << 16u),
		COLLISION_17 = uint32(1u << 17u),
		COLLISION_18 = uint32(1u << 18u),
		COLLISION_19 = uint32(1u << 19u),
		COLLISION_20 = uint32(1u << 20u),
		COLLISION_21 = uint32(1u << 21u),
		COLLISION_22 = uint32(1u << 22u),
		COLLISION_23 = uint32(1u << 23u),
		COLLISION_24 = uint32(1u << 24u),
		COLLISION_25 = uint32(1u << 25u),
		COLLISION_26 = uint32(1u << 26u),
		COLLISION_27 = uint32(1u << 27u),
		COLLISION_28 = uint32(1u << 28u),
		VEHICLE = uint32(1u << 29u),
		TERRAIN = uint32(1u << 30u),
		RAGDOLL = uint32(1u << 31u),
		ALL = uint32(0xFFFFFFFF),
	};
	DEFINE_BITWISE_OPERATORS_FOR_ENUM(CollisionType)

	constexpr static const uint32 TYPES_COUNT = 32u;

	enum class FilterData : uint32
	{
		HAS_FILTER_DATA_FLAG = (1 << 0)
	};
}