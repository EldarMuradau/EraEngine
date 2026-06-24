#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include "physics/destructions/destruction_types.h"
#include "physics/destructions/assets/destructible_box_generator.h"

namespace era_engine::physics
{
	class ERA_PHYSICS_API DestructibleBoxAsset : public DestructibleAsset
	{
	public:
		struct Desc
		{
			DestructibleBoxGenerator::Settings generator_settings;
			float static_height;
			bool joint_all_bonds;
		};

		DestructibleBoxAsset(const Desc& desc);
		~DestructibleBoxAsset() override;

		DestructibleFamilyPtr create_family(World* world, Nv::Blast::ExtPxManager& manager, const ActorDesc& desc) override;

	private:
		physx::PxConvexMesh* box_mesh = nullptr;
		GeneratorAsset	generator_asset;
	};
}