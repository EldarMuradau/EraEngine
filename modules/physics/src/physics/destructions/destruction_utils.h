#pragma once

#include "physics_api.h"

#include "physics/destructions/destruction_types.h"

#include <core/math.h>

#include <asset/model_asset.h>

#include <NvBlastTypes.h>
#include <NvBlastExtAuthoringFractureTool.h>
#include <NvBlastExtAuthoring.h>

namespace Nv
{
	namespace Blast
	{
		struct Triangle;
		class VoronoiSitesGenerator;
	}
}

namespace era_engine::physics
{
	class ERA_PHYSICS_API DestructionUtils final
	{
	public:
		DestructionUtils() = delete;

		static std::vector<uint32> generate_indices(Nv::Blast::Triangle* triangles, size_t nb_triangles);

		static NvBlastID generate_id_from_string(const char* str);
	};

	class BlastRandomGenerator : public Nv::Blast::RandomGeneratorBase
	{
	public:
		BlastRandomGenerator();

		float getRandomValue() override;

		void seed(int32_t seed) override;

	private:
		int32 seed_result = 5489U;

		std::random_device rd;
        std::unique_ptr<std::mt19937> eng;
	};

	struct VoronoiSitesGenerator
	{
		VoronoiSitesGenerator(ref<NvMesh> mesh);
		~VoronoiSitesGenerator();

		ref<BlastRandomGenerator> rnd_gen = nullptr;
		Nv::Blast::VoronoiSitesGenerator* generator = nullptr;
	};

	struct FractureTool
	{
		FractureTool();
		~FractureTool();

		Nv::Blast::FractureTool* fracture = nullptr;
	};

	class ERA_PHYSICS_API FractureUtils final
	{
	public:
		FractureUtils() = delete;

		static std::vector<std::pair<ref<SubmeshAsset>, ref<NvMesh>>> fracture_nvmesh_into_submeshes(uint32 total_chunks, 
			ref<NvMesh> mesh, 
			bool replace = false, 
			uint32 chunk_id = 0);
	};
}