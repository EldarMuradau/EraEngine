#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include <NvBlastTypes.h>

#include <shaders/NvBlastExtDamageShaders.h>

namespace Nv
{
    namespace Blast
    {
        struct Mesh;

        class TkGroup;
        class TkAsset;
        class TkFramework;
        class ExtPxManager;

        class ExtPxAsset;
    }
}

class ExtDamageAccelerator;

namespace era_engine
{
    struct SubmeshAsset;
}

namespace era_engine::physics
{
    class ERA_PHYSICS_API NvMesh final
    {
    public:
		NvMesh(const std::vector<physx::PxVec3>& verts,
			const std::vector<physx::PxVec3>& norms,
			const std::vector<physx::PxVec2>& uvis,
			const std::vector<uint32_t>& inds);
        NvMesh(Nv::Blast::Mesh* in_mesh);
        NvMesh();

        ~NvMesh();

        void release();
        void clean_mesh();

        ref<SubmeshAsset> create_render_submesh() const;

        Nv::Blast::TkAsset* create_tk_asset(Nv::Blast::TkFramework& framework) const;

    public:
        std::vector<physx::PxVec3> vertices;
        std::vector<physx::PxVec3> normals;
        std::vector<physx::PxVec2> uvs;
        std::vector<uint32> indices;

        Nv::Blast::Mesh* mesh = nullptr;
    };

    class DestructibleFamily;
    typedef std::shared_ptr<DestructibleFamily> DestructibleFamilyPtr;

	class ERA_PHYSICS_API DestructibleAsset
	{
	public:
        struct ActorDesc
        {
            NvBlastID id;
            physx::PxTransform transform;
            Nv::Blast::TkGroup* group = nullptr;
        };

		DestructibleAsset();
		virtual ~DestructibleAsset();

        size_t get_asset_size() const;

        virtual DestructibleFamilyPtr createFamily(Nv::Blast::ExtPxManager& manager, const ActorDesc& desc) = 0;

        float bond_health_max = 1.0f;
        float support_chunk_health_max = 1.0f;

        Nv::Blast::ExtPxAsset* px_asset = nullptr;
        NvBlastExtDamageAccelerator* damage_accelerator = nullptr;

    protected:
        void initialize();

	protected:

		friend class DestructionUtils;
		friend class DestructionSystem;
	};
}