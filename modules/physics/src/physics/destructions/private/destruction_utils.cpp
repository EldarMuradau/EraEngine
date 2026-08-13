#include "physics/destructions/destruction_utils.h"

#include <NvBlastGlobals.h>
#include <extensions/authoringCommon/NvBlastExtAuthoringMesh.h>
#include <extensions/authoringCommon/NvBlastExtAuthoringTypes.h>
#include <core/log.h>

namespace era_engine::physics
{
    std::vector<uint32> DestructionUtils::generate_indices(Nv::Blast::Triangle* triangles, size_t nb_triangles)
    {
        struct Vertex
        {
            vec3 position;
            vec3 normal;
            vec2 uv;

            bool operator==(const Vertex& other) const
            {
                return fuzzy_equals(position, other.position) && 
                    fuzzy_equals(normal, other.normal) &&
                    fuzzy_equals(uv, other.uv);
            }

            bool operator!=(const Vertex& other) const
            {
                return !operator==(other);
            }
        };

        std::vector<Vertex> vertices;

        std::vector<uint32> indices;
        vertices.reserve(nb_triangles * 3);

        for (size_t i = 0; i < nb_triangles; i++)
        {
            auto& triangle = triangles[i];

            {
                Vertex vertex = {
                    vec3(triangle.a.p.x, triangle.a.p.y, triangle.a.p.z),
                    vec3(triangle.a.n.x, triangle.a.n.y, triangle.a.n.z),
                    vec2(triangle.a.uv->x, triangle.a.uv->y) };

                auto it = std::find(vertices.begin(), vertices.end(), vertex);
                if (it != vertices.end())
                {
                    // Vertex already exists in the vertices vector, so we just need to add the index.
                    indices.push_back(std::distance(vertices.begin(), it));
                }
                else
                {
                    // New vertex, so we need to add it to the vertices vector and add the index.
                    vertices.push_back(vertex);
                    indices.push_back(vertices.size() - 1);
                }
            }

            {
                Vertex vertex = {
                    vec3(triangle.b.p.x, triangle.b.p.y, triangle.b.p.z),
                    vec3(triangle.b.n.x, triangle.b.n.y, triangle.b.n.z),
                    vec2(triangle.b.uv->x, triangle.b.uv->y) };

                auto it = std::find(vertices.begin(), vertices.end(), vertex);
                if (it != vertices.end())
                {
                    // Vertex already exists in the vertices vector, so we just need to add the index.
                    indices.push_back(std::distance(vertices.begin(), it));
                }
                else
                {
                    // New vertex, so we need to add it to the vertices vector and add the index.
                    vertices.push_back(vertex);
                    indices.push_back(vertices.size() - 1);
                }
            }

            {
                Vertex vertex = {
                    vec3(triangle.c.p.x, triangle.c.p.y, triangle.c.p.z),
                    vec3(triangle.c.n.x, triangle.c.n.y, triangle.c.n.z),
                    vec2(triangle.c.uv->x, triangle.c.uv->y) };

                auto it = std::find(vertices.begin(), vertices.end(), vertex);
                if (it != vertices.end())
                {
                    // Vertex already exists in the vertices vector, so we just need to add the index.
                    indices.push_back(std::distance(vertices.begin(), it));
                }
                else
                {
                    // New vertex, so we need to add it to the vertices vector and add the index.
                    vertices.push_back(vertex);
                    indices.push_back(vertices.size() - 1);
                }
            }
        }

        return indices;
    }

    NvBlastID DestructionUtils::generate_id_from_string(const char* str)
    {
        uint32 h[4] = { 5381, 5381, 5381, 5381 };

        uint32 i = 0;

        for (const char* ptr = str; *ptr; i = ((i + 1) & 3), ++ptr)
        {
            h[i] = ((h[i] << 5) + h[i]) ^ static_cast<uint32>(*ptr);
        }

        return *reinterpret_cast<NvBlastID*>(h);
    }

    BlastRandomGenerator::BlastRandomGenerator()
    {
        eng = std::make_unique<std::mt19937>(rd());
        eng->seed(seed_result);
    }

    float BlastRandomGenerator::getRandomValue()
    {
        std::uniform_real_distribution<float> distr(0.0f, 1.0f);
        return distr(*eng.get());
    }

    void BlastRandomGenerator::seed(int32_t seed)
    {
        seed_result = seed;
        eng->seed(seed_result);
    }

    VoronoiSitesGenerator::VoronoiSitesGenerator(ref<NvMesh> mesh)
    {
        rnd_gen = make_ref<BlastRandomGenerator>();
        generator = NvBlastExtAuthoringCreateVoronoiSitesGenerator(mesh->mesh, reinterpret_cast<Nv::Blast::RandomGeneratorBase*>(rnd_gen.get()));
    }

    VoronoiSitesGenerator::~VoronoiSitesGenerator()
    {
        RELEASE_PTR(generator)
    }

    FractureTool::FractureTool()
    {
        fracture = NvBlastExtAuthoringCreateFractureTool();
    }

    FractureTool::~FractureTool()
    {
        PX_RELEASE(fracture)
    }

    std::vector<std::pair<ref<SubmeshAsset>, ref<NvMesh>>> FractureUtils::fracture_nvmesh_into_submeshes(uint32 total_chunks, 
        ref<NvMesh> mesh, 
        bool replace /*= false*/, 
        uint32 chunk_id/* = 0*/)
    {
        FractureTool fracture_tool = FractureTool();

        fracture_tool.fracture->setRemoveIslands(true);
        fracture_tool.fracture->setSourceMeshes(&mesh->mesh, 1);

        if (fracture_tool.fracture->isMeshContainOpenEdges(mesh->mesh))
        {
            LOG_WARNING("NvBlast> Mesh contains open edges!");
            //return {};
        }

        VoronoiSitesGenerator generator = VoronoiSitesGenerator(mesh);
        generator.generator->setBaseMesh(mesh->mesh);

        generator.generator->uniformlyGenerateSitesInMesh(total_chunks);

        const NvcVec3* sites;
        const size_t nb_sites = generator.generator->getVoronoiSites(sites);

        const int result = fracture_tool.fracture->voronoiFracturing(chunk_id, nb_sites, sites, replace);
        if (result != 0)
        {
            LOG_ERROR("NvBlast> Failed to fracture mesh!");
            return {};
        }

        fracture_tool.fracture->finalizeFracturing();

        const size_t mesh_count = fracture_tool.fracture->getChunkCount();

        std::vector<std::pair<ref<SubmeshAsset>, ref<NvMesh>>> meshes;
        meshes.reserve(mesh_count);

        std::vector<std::vector<Nv::Blast::Triangle>> chunk_meshes;
        chunk_meshes.reserve(mesh_count);

        Nv::Blast::Triangle* trigs = nullptr;

        for (size_t i = 1; i < mesh_count; ++i)
        {
            const uint32 nb_trigs = fracture_tool.fracture->getBaseMesh(i, trigs);

            std::vector<Nv::Blast::Triangle> trig_list;
            trig_list.reserve(nb_trigs);

            for (size_t j = 0; j < nb_trigs; ++j)
            {
                trig_list.push_back(trigs[j]);
            }

            chunk_meshes.push_back(trig_list);
        }

        for (size_t i = 0; i < chunk_meshes.size(); ++i)
        {
            std::vector<Nv::Blast::Triangle>& chunk = chunk_meshes[i];

            const size_t chunk_size = chunk.size();
            std::vector<physx::PxVec3> pos;
            pos.reserve(chunk_size * 3);

            std::vector<physx::PxVec3> norm;
            norm.reserve(chunk_size * 3);

            std::vector<physx::PxVec2> tex;
            tex.reserve(chunk_size * 3);

            std::vector<uint32> indexs;
            indexs.reserve(chunk_size * 3);

            uint32 index = 0;
            for (size_t j = 0; j < chunk_size; ++j)
            {
                pos.push_back({ chunk[j].a.p.x, chunk[j].a.p.y, chunk[j].a.p.z });
                pos.push_back({ chunk[j].b.p.x, chunk[j].b.p.y, chunk[j].b.p.z });
                pos.push_back({ chunk[j].c.p.x, chunk[j].c.p.y, chunk[j].c.p.z });

                norm.push_back({ chunk[j].a.n.x, chunk[j].a.n.y, chunk[j].a.n.z });
                norm.push_back({ chunk[j].b.n.x, chunk[j].b.n.y, chunk[j].b.n.z });
                norm.push_back({ chunk[j].c.n.x, chunk[j].c.n.y, chunk[j].c.n.z });

                tex.push_back({ chunk[j].a.uv[0].x, chunk[j].a.uv[0].y });
                tex.push_back({ chunk[j].b.uv[0].x, chunk[j].b.uv[0].y });
                tex.push_back({ chunk[j].c.uv[0].x, chunk[j].c.uv[0].y });

                indexs.push_back(index++);
                indexs.push_back(index++);
                indexs.push_back(index++);
            }

            ref<NvMesh> mesh = make_ref<NvMesh>(pos, norm, tex, indexs);

            ref<SubmeshAsset> chunk_mesh = mesh->create_render_submesh();

            meshes.push_back(std::make_pair(std::move(chunk_mesh), std::move(mesh)));
        }

        return meshes;
    }
}