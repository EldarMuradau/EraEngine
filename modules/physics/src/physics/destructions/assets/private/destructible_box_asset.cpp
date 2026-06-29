#include "physics/destructions/assets/destructible_box_asset.h"
#include "physics/core/physics.h"
#include "physics/destructions/families/box_destructible_family.h"
#include "physics/destructions/blast_physx/NvBlastExtPxAsset.h"
#include "physics/destructions/blast_physx/NvBlastExtPxManager.h"

#include <NvBlastTk.h>

#include <extensions/authoring/NvBlastExtAuthoring.h>
#include <extensions/authoring/NvBlastExtAuthoringMeshCleaner.h>
#include <extensions/authoringCommon/NvBlastExtAuthoringTypes.h>
#include <extensions/authoringCommon/NvBlastExtAuthoringMesh.h>

namespace era_engine::physics
{
	DestructibleBoxAsset::DestructibleBoxAsset(const Desc& desc)
		: DestructibleAsset()
	{
		using namespace physx;
		using namespace Nv::Blast;

		DestructibleBoxGenerator::generate(generator_asset, desc.generator_settings);

		ExtPxAssetDesc asset_desc;
		asset_desc.chunkDescs = generator_asset.solver_chunks.data();
		asset_desc.chunkCount = (uint32)generator_asset.solver_chunks.size();
		asset_desc.bondDescs = generator_asset.solver_bonds.data();
		asset_desc.bondCount = (uint32)generator_asset.solver_bonds.size();

		std::vector<uint8_t> bondFlags(asset_desc.bondCount);
		std::fill(bondFlags.begin(), bondFlags.end(), desc.joint_all_bonds ? 1 : 0);
		asset_desc.bondFlags = bondFlags.data();

		PxVec3 vertices[8] = { { -1, -1, -1 }, { -1, -1, 1 }, { -1, 1, -1 }, { -1, 1, 1 }, { 1, -1, -1 }, { 1, -1, 1 }, { 1, 1, -1 }, { 1, 1, 1 } };
		PxConvexMeshDesc convex_mesh_desc;
		convex_mesh_desc.points.count = 8;
		convex_mesh_desc.points.data = vertices;
		convex_mesh_desc.points.stride = sizeof(PxVec3);
		convex_mesh_desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

		PxCookingParams cooking_params = PxCookingParams(PhysicsEngine::get_physics_core()->get_tolerance_scale());

		if(PhysicsEngine::get_physics_core()->is_gpu())
		{
			cooking_params.buildGPUData = true;
		}
		cooking_params.convexMeshCookingType = PxConvexMeshCookingType::eQUICKHULL;
		cooking_params.meshPreprocessParams = PxMeshPreprocessingFlag::eENABLE_INERTIA;
		box_mesh = PxCreateConvexMesh(cooking_params, convex_mesh_desc, PhysicsEngine::get_physics_core()->get_physics()->getPhysicsInsertionCallback());

		const uint32 chunkCount = (uint32_t)generator_asset.solver_chunks.size();
		std::vector<ExtPxAssetDesc::ChunkDesc> px_chunks(chunkCount);
		std::vector<ExtPxAssetDesc::SubchunkDesc> pxSubchunks;
		pxSubchunks.reserve(chunkCount);

		for (uint32 i = 0; i < generator_asset.solver_chunks.size(); i++)
		{
			uint32 chunk_id = generator_asset.solver_chunks[i].userData;
			GeneratorAsset::BlastChunkCube& cube = generator_asset.chunks[chunk_id];
			PxVec3 position = *reinterpret_cast<PxVec3*>(&cube.position);
			PxVec3 extents = *reinterpret_cast<PxVec3*>(&cube.extents);
			ExtPxAssetDesc::ChunkDesc& chunk = px_chunks[chunk_id];
			ExtPxAssetDesc::SubchunkDesc subchunk =
			{
				PxTransform(position),
				PxConvexMeshGeometry(box_mesh, PxMeshScale(extents / 2))
			};
			pxSubchunks.push_back(subchunk);
			chunk.subchunks = &pxSubchunks.back();
			chunk.subchunkCount = 1;
			chunk.isStatic = (position.y - (extents.y - desc.generator_settings.extents.y) / 2) <= desc.static_height;
		}

		asset_desc.pxChunks = px_chunks.data();
		px_asset = ExtPxAsset::create(asset_desc, *PhysicsEngine::get_physics_core()->get_blast_core()->tk_framework);

		initialize();
	}

	DestructibleBoxAsset::~DestructibleBoxAsset()
	{
		box_mesh->release();
		px_asset->release();
	}

	DestructibleFamilyPtr DestructibleBoxAsset::create_family(World* world, Nv::Blast::ExtPxManager& manager, const ActorDesc& desc)
	{
		return DestructibleFamilyPtr(new BoxDestructibleFamily(world, manager, this, desc));
	}

}