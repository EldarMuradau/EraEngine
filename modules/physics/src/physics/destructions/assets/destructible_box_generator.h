#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include "physics/destructions/destruction_types.h"

#include <core/math.h>

namespace era_engine::physics
{
	class ERA_PHYSICS_API GeneratorAsset
	{
	public:
		struct BlastChunkCube
		{
			BlastChunkCube(const vec3& _position, const vec3& _extents)
			{
				position = _position;
				extents = _extents;
			}

			vec3 position;
			vec3 extents;
		};

		std::vector<NvBlastChunkDesc> solver_chunks;
		std::vector<NvBlastBondDesc> solver_bonds;
		std::vector<BlastChunkCube> chunks;
		vec3 extents;
	};

	class ERA_PHYSICS_API DestructibleBoxGenerator
	{
	public:
		struct DepthInfo
		{
			DepthInfo(const vec3& slices = vec3(1, 1, 1), NvBlastChunkDesc::Flags flag_ = NvBlastChunkDesc::Flags::NoFlags)
				: slices_per_axis(slices), flag(flag_) {}

			vec3 slices_per_axis;
			NvBlastChunkDesc::Flags flag;
		};

		enum BondFlags
		{
			NO_BONDS = 0,
			X_BONDS = 1 << 0,
			Y_BONDS = 1 << 1,
			Z_BONDS = 1 << 2,
			X_PLUS_WORLD_BONDS = 1 << 3,
			X_MINUS_WORLD_BONDS = 1 << 4,
			Y_PLUS_WORLD_BONDS = 1 << 5,
			Y_MINUS_WORLD_BONDS = 1 << 6,
			Z_PLUS_WORLD_BONDS = 1 << 7,
			Z_MINUS_WORLD_BONDS = 1 << 8,
			ALL_INTERNAL_BONDS = X_BONDS | Y_BONDS | Z_BONDS
		};

		struct Settings
		{
			Settings() : bondFlags(BondFlags::ALL_INTERNAL_BONDS) {}

			std::vector<DepthInfo> depths;
			vec3 extents;
			BondFlags bondFlags;
		};

		static void generate(GeneratorAsset& asset, const Settings& settings);
	private:
		static void fill_bond_desc(std::vector<NvBlastBondDesc>& bond_descs, 
			uint32 id0, 
			uint32 id1, 
			const vec3& pos0, 
			const vec3& pos1, 
			const vec3& size, 
			float area);
	};

	inline DestructibleBoxGenerator::BondFlags operator | (DestructibleBoxGenerator::BondFlags a, DestructibleBoxGenerator::BondFlags b)
	{
		return static_cast<DestructibleBoxGenerator::BondFlags>(static_cast<int32>(a) | static_cast<int32>(b));
	}
}