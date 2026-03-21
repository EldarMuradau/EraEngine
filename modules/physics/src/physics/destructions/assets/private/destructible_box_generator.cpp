#include "physics/destructions/assets/destructible_box_asset.h"

#include <NvBlastTk.h>
#include <NvBlast.h>

namespace era_engine::physics
{
	void DestructibleBoxGenerator::generate(GeneratorAsset& asset, const Settings& settings)
	{
		asset.solver_chunks.clear();
		asset.solver_bonds.clear();
		asset.chunks.clear();

		std::vector<uint32> depth_start_ids;
		std::vector<vec3> depth_slices_per_axis_total;
		uint32 current_id = 0;
		vec3 extents = settings.extents;
		asset.extents = extents;

		// Iterate over depths and create children.
		for (uint32 depth = 0; depth < settings.depths.size(); depth++)
		{
			vec3 slices_per_axis = settings.depths[depth].slices_per_axis;
			vec3 slices_per_axis_total = (depth == 0) ? slices_per_axis : slices_per_axis * (depth_slices_per_axis_total[depth - 1]);
			depth_slices_per_axis_total.push_back(slices_per_axis_total);

			depth_start_ids.push_back(current_id);

			extents.x /= slices_per_axis.x;
			extents.y /= slices_per_axis.y;
			extents.z /= slices_per_axis.z;

			for (uint32 z = 0; z < (uint32)slices_per_axis_total.z; ++z)
			{
				uint32 parent_z = z / (uint32)slices_per_axis.z;
				for (uint32 y = 0; y < (uint32)slices_per_axis_total.y; ++y)
				{
					uint32 parent_y = y / (uint32)slices_per_axis.y;
					for (uint32 x = 0; x < (uint32)slices_per_axis_total.x; ++x)
					{
						uint32 parent_x = x / (uint32)slices_per_axis.x;
						uint32 parent_id = depth == 0 ? UINT32_MAX :
							depth_start_ids[depth - 1] + parent_x + (uint32)depth_slices_per_axis_total[depth - 1].x * (parent_y + (uint32)depth_slices_per_axis_total[depth - 1].y * parent_z);

						vec3 position;
						position.x = ((float)x - (slices_per_axis_total.x / 2) + 0.5f) * extents.x;
						position.y = ((float)y - (slices_per_axis_total.y / 2) + 0.5f) * extents.y;
						position.z = ((float)z - (slices_per_axis_total.z / 2) + 0.5f) * extents.z;

						NvBlastChunkDesc chunk_desc{};
						memcpy(chunk_desc.centroid, &position.x, 3 * sizeof(float));
						chunk_desc.volume = extents.x * extents.y * extents.z;
						chunk_desc.flags = settings.depths[depth].flag;
						chunk_desc.userData = current_id++;
						chunk_desc.parentChunkDescIndex = parent_id;
						asset.solver_chunks.push_back(chunk_desc);

						if (settings.depths[depth].flag & NvBlastChunkDesc::Flags::SupportFlag)
						{
							// Internal bonds

							// x-neighbor
							if (x > 0 && (settings.bondFlags & BondFlags::X_BONDS))
							{
								vec3 x_neighbor_position = position - vec3(extents.x, 0, 0);
								uint32 neighbor_id = chunk_desc.userData - 1;
								fill_bond_desc(asset.solver_bonds, chunk_desc.userData, neighbor_id, position, x_neighbor_position, extents, extents.y * extents.z);
							}

							// y-neighbor
							if (y > 0 && (settings.bondFlags & BondFlags::Y_BONDS))
							{
								vec3 y_neighbor_position = position - vec3(0, extents.y, 0);
								uint32 neighbor_id = chunk_desc.userData - (uint32)slices_per_axis_total.x;
								fill_bond_desc(asset.solver_bonds, chunk_desc.userData, neighbor_id, position, y_neighbor_position, extents, extents.z * extents.x);
							}

							// z-neighbor
							if (z > 0 && (settings.bondFlags & BondFlags::Z_BONDS))
							{
								vec3 z_neighbor_position = position - vec3(0, 0, extents.z);
								uint32 neighbor_id = chunk_desc.userData - (uint32)slices_per_axis_total.x * (uint32)slices_per_axis_total.y;
								fill_bond_desc(asset.solver_bonds, chunk_desc.userData, neighbor_id, position, z_neighbor_position, extents, extents.x * extents.y);
							}

							// World bonds (only one per chunk is enough, otherwise they will be removed as duplicated, thus 'else if').

							// -x world bond
							if (x == 0 && (settings.bondFlags & BondFlags::X_MINUS_WORLD_BONDS))
							{
								vec3 x_neighbor_position = position - vec3(extents.x, 0, 0);
								fill_bond_desc(asset.solver_bonds, chunk_desc.userData, UINT32_MAX, position, x_neighbor_position, extents, extents.y * extents.z);
							}
							// +x world bond
							else if (x == slices_per_axis_total.x - 1 && (settings.bondFlags & BondFlags::X_PLUS_WORLD_BONDS))
							{
								vec3 x_neighbor_position = position + vec3(extents.x, 0, 0);
								fill_bond_desc(asset.solver_bonds, chunk_desc.userData, UINT32_MAX, position, x_neighbor_position, extents, extents.y * extents.z);
							}
							// -y world bond
							else if (y == 0 && (settings.bondFlags & BondFlags::Y_MINUS_WORLD_BONDS))
							{
								vec3 y_neighbor_position = position - vec3(0, extents.y, 0);
								fill_bond_desc(asset.solver_bonds, chunk_desc.userData, UINT32_MAX, position, y_neighbor_position, extents, extents.z * extents.x);
							}
							// +y world bond
							else if (y == slices_per_axis_total.y - 1 && (settings.bondFlags & BondFlags::Y_PLUS_WORLD_BONDS))
							{
								vec3 y_neighbor_position = position + vec3(0, extents.y, 0);
								fill_bond_desc(asset.solver_bonds, chunk_desc.userData, UINT32_MAX, position, y_neighbor_position, extents, extents.z * extents.x);
							}
							// -z world bond
							else if (z == 0 && (settings.bondFlags & BondFlags::Z_MINUS_WORLD_BONDS))
							{
								vec3 z_neighbor_position = position - vec3(0, 0, extents.z);
								fill_bond_desc(asset.solver_bonds, chunk_desc.userData, UINT32_MAX, position, z_neighbor_position, extents, extents.x * extents.y);
							}
							// +z world bond
							else if (z == slices_per_axis_total.z - 1 && (settings.bondFlags & BondFlags::Z_PLUS_WORLD_BONDS))
							{
								vec3 z_neighbor_position = position + vec3(0, 0, extents.z);
								fill_bond_desc(asset.solver_bonds, chunk_desc.userData, UINT32_MAX, position, z_neighbor_position, extents, extents.x * extents.y);
							}
						}

						asset.chunks.push_back(GeneratorAsset::BlastChunkCube(position, extents/*isStatic*/));
					}
				}
			}
		}

		// Reorder chunks
		std::vector<uint32> chunk_reorder_map(asset.solver_chunks.size());
		std::vector<char> scratch(asset.solver_chunks.size() * sizeof(NvBlastChunkDesc));
		NvBlastBuildAssetDescChunkReorderMap(chunk_reorder_map.data(), 
			asset.solver_chunks.data(), 
			(uint32)asset.solver_chunks.size(), 
			scratch.data(), 
			nullptr);

		std::vector<GeneratorAsset::BlastChunkCube> chunksTemp = asset.chunks;
		for (uint32 i = 0; i < chunk_reorder_map.size(); ++i)
		{
			asset.chunks[chunk_reorder_map[i]] = chunksTemp[i];
		}

		NvBlastApplyAssetDescChunkReorderMapInPlace(asset.solver_chunks.data(), 
			(uint32)asset.solver_chunks.size(), 
			asset.solver_bonds.data(), 
			(uint32)asset.solver_bonds.size(), 
			chunk_reorder_map.data(), 
			true, 
			scratch.data(), 
			nullptr);

	}

	void DestructibleBoxGenerator::fill_bond_desc(std::vector<NvBlastBondDesc>& bond_descs, uint32 id0, uint32 id1, const vec3& pos0, const vec3& pos1, const vec3& size, float area)
	{
		NV_UNUSED(size);

		NvBlastBondDesc bondDesc{};
		bondDesc.chunkIndices[0] = id0;
		bondDesc.chunkIndices[1] = id1;
		bondDesc.bond.area = area;

		vec3 centroid = (pos0 + pos1) * 0.5f;
		bondDesc.bond.centroid[0] = centroid.x;
		bondDesc.bond.centroid[1] = centroid.y;
		bondDesc.bond.centroid[2] = centroid.z;

		vec3 normal = normalize(pos0 - pos1);
		bondDesc.bond.normal[0] = normal.x;
		bondDesc.bond.normal[1] = normal.y;
		bondDesc.bond.normal[2] = normal.z;

		bond_descs.push_back(bondDesc);
	}
}