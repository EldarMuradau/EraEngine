#include "physics/destructions/assets/destructible_model_asset.h"
#include "physics/destructions/blast_physx/NvBlastExtPxAsset.h"
#include "physics/destructions/blast_physx/NvBlastExtPxManager.h"

#include <rendering/pbr_material.h>

#include <asset/model_asset.h>
#include <asset/mesh_postprocessing.h>

#include <NvBlastTk.h>

#include <extensions/authoring/NvBlastExtAuthoring.h>
#include <extensions/authoring/NvBlastExtAuthoringMeshCleaner.h>
#include <extensions/authoringCommon/NvBlastExtAuthoringTypes.h>
#include <extensions/authoringCommon/NvBlastExtAuthoringMesh.h>

namespace era_engine::physics
{
	DestructibleModelAsset::DestructibleModelAsset()
		: DestructibleAsset()
	{
		initialize();
	}

	DestructibleModelAsset::~DestructibleModelAsset()
	{
	}

}