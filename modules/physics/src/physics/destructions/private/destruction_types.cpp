#include "physics/destructions/destruction_types.h"
#include "physics/destructions/blast_physx/NvBlastExtPxAsset.h"
#include "physics/destructions/blast_physx/NvBlastExtPxManager.h"

#include <rendering/pbr_material.h>

#include <asset/model_asset.h>
#include <asset/mesh_postprocessing.h>

#include <NvBlastTk.h>

#include <extensions/authoring/NvBlastExtAuthoring.h>
#include <extensions/authoring/NvBlastExtAuthoringFractureTool.h>
#include <extensions/authoring/NvBlastExtAuthoringMeshCleaner.h>
#include <extensions/authoringCommon/NvBlastExtAuthoringTypes.h>
#include <extensions/authoringCommon/NvBlastExtAuthoringMesh.h>
#include <extensions/shaders/NvBlastExtDamageShaders.h>

namespace era_engine::physics
{
	DestructibleAsset::DestructibleAsset()
	{
	}

	DestructibleAsset::~DestructibleAsset()
	{
        if (damage_accelerator)
        {
            damage_accelerator->release();
        }
	}

	size_t DestructibleAsset::get_asset_size() const
	{
		return px_asset->getTkAsset().getDataSize();
	}

	void DestructibleAsset::initialize()
	{
		using namespace Nv::Blast;

		// Calc max healths.
		const auto& actor_desc = px_asset->getDefaultActorDesc();
		if (actor_desc.initialBondHealths)
		{
			bond_health_max = FLT_MIN;
			const uint32 bond_count = px_asset->getTkAsset().getBondCount();
			for (uint32 i = 0; i < bond_count; ++i)
			{
				bond_health_max = std::max<float>(bond_health_max, actor_desc.initialBondHealths[i]);
			}
		}
		else
		{
			bond_health_max = actor_desc.uniformInitialBondHealth;
		}

		if (actor_desc.initialSupportChunkHealths)
		{
			support_chunk_health_max = FLT_MIN;
			const uint32 node_count = px_asset->getTkAsset().getGraph().nodeCount;
			for (uint32 i = 0; i < node_count; ++i)
			{
				support_chunk_health_max = std::max<float>(support_chunk_health_max, actor_desc.initialSupportChunkHealths[i]);
			}
		}
		else
		{
			support_chunk_health_max = actor_desc.uniformInitialLowerSupportChunkHealth;
		}

		damage_accelerator = NvBlastExtDamageAcceleratorCreate(px_asset->getTkAsset().getAssetLL(), 3);
	}

	NvMesh::NvMesh(const std::vector<physx::PxVec3>& verts,
		const std::vector<physx::PxVec3>& norms,
		const std::vector<physx::PxVec2>& uvis,
		const std::vector<uint32_t>& inds)
		: vertices(verts),
		normals(norms),
		uvs(uvis),
		indices(inds)
	{
		using namespace Nv::Blast;

		mesh = NvBlastExtAuthoringCreateMesh((NvcVec3*)vertices.data(), 
			(NvcVec3*)normals.data(), 
			(NvcVec2*)uvs.data(),
			vertices.size(), 
			indices.data(), 
			indices.size());
	}

	NvMesh::NvMesh()
	{
		using namespace Nv::Blast;

		if (!vertices.empty() &&
			!indices.empty())
		{
			mesh = NvBlastExtAuthoringCreateMesh((NvcVec3*)vertices.data(),
				(NvcVec3*)normals.data(),
				(NvcVec2*)uvs.data(),
				vertices.size(),
				indices.data(),
				indices.size());
		}
	}

	NvMesh::NvMesh(Nv::Blast::Mesh* in_mesh)
	{
		using namespace physx;
		using namespace Nv::Blast;

		mesh = in_mesh;

		const size_t nb_verts = mesh->getVerticesCount();
		const Vertex* verts = mesh->getVertices();

		for (size_t i = 0; i < nb_verts; ++i)
		{
			vertices.push_back(PxVec3(verts[i].p.x, verts[i].p.y, verts[i].p.z));
			uvs.push_back(PxVec2(verts[i].uv->x, verts[i].uv->y));

			normals.push_back(PxVec3(verts[i].n.x, verts[i].n.y, verts[i].n.z));
		}
	}

	NvMesh::~NvMesh()
	{
		release();
	}

	void NvMesh::release()
	{
		clean_mesh();

		vertices.clear();
		normals.clear();
		uvs.clear();
		indices.clear();

		RELEASE_PTR(mesh);
	}

	void NvMesh::clean_mesh()
	{
		using namespace Nv::Blast;

		MeshCleaner* cleaner = NvBlastExtAuthoringCreateMeshCleaner();
		mesh = cleaner->cleanMesh(mesh);

		RELEASE_PTR(cleaner)
	}

	ref<SubmeshAsset> NvMesh::create_render_submesh() const
	{
		using namespace physx;

		ref<SubmeshAsset> asset = make_ref<SubmeshAsset>();

		for (auto& vert : vertices)
		{
			asset->positions.push_back(create_vec3(vert));
		}

		for (auto& uv : uvs)
		{
			asset->uvs.push_back(create_vec2(uv));
		}

		for (auto& norm : normals)
		{
			asset->normals.push_back(create_vec3(norm));
		}

		for (uint32 i = 0; i < indices.size(); i += 3)
		{
			asset->triangles.push_back(indexed_triangle32{
				(uint32)indices[i + 0],
				(uint32)indices[i + 1],
				(uint32)indices[i + 2] });
		}

		generateNormalsAndTangents(asset, MESH_FLAG_DEFAULT);

		return asset;
	}

	static uint32 find_chunk_for_facet(const std::vector<std::vector<uint32>>& connected_components, uint32 facet_index)
	{
		for (uint32 chunk_idx = 0; chunk_idx < connected_components.size(); ++chunk_idx)
		{
			for (uint32 facet_in_chunk : connected_components[chunk_idx])
			{
				if (facet_in_chunk == facet_index)
				{
					return chunk_idx;
				}
			}
		}

		return 0;
	}

    static NvcVec3 calculate_face_normal(const Nv::Blast::Vertex* vertices, const Nv::Blast::Edge* edges, const Nv::Blast::Facet& facet)
    {
        using namespace Nv::Blast;

        const Vertex& v0 = vertices[facet.firstEdgeNumber];
        const Vertex& v1 = vertices[edges[facet.firstEdgeNumber].s];
        const Vertex& v2 = vertices[edges[facet.firstEdgeNumber + 1].s];

        NvcVec3 edge1 = { v1.p.x - v0.p.x, v1.p.y - v0.p.y, v1.p.z - v0.p.z };
        NvcVec3 edge2 = { v2.p.x - v0.p.x, v2.p.y - v0.p.y, v2.p.z - v0.p.z };

        NvcVec3 normal = {
            edge1.y * edge2.z - edge1.z * edge2.y,
            edge1.z * edge2.x - edge1.x * edge2.z,
            edge1.x * edge2.y - edge1.y * edge2.x
        };

        float length = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
        if (length > 0.0f) {
            normal.x /= length;
            normal.y /= length;
            normal.z /= length;
        }

        return normal;
    }


	Nv::Blast::TkAsset* NvMesh::create_tk_asset(Nv::Blast::TkFramework& framework) const
	{
		using namespace Nv::Blast;

		if (mesh == nullptr)
		{
			return nullptr;
		}

        const uint32 facet_count = mesh->getFacetCount();
        const uint32 edge_count = mesh->getEdgesCount();

        const Vertex* vertices = mesh->getVertices();
        const Facet* facets = mesh->getFacetsBuffer();
        const Edge* edges = mesh->getEdges();

        NvcVec3 mesh_centroid;
        float mesh_volume = mesh->getMeshVolumeAndCentroid(mesh_centroid);

        std::vector<std::vector<uint32>> connected_components;
        std::vector<bool> visited_facets(facet_count, false);

        std::vector<std::vector<uint32>> facet_adjacency(facet_count);
        for (uint32 i = 0; i < edge_count; ++i)
        {
            const Edge& edge = edges[i];
            if (edge.e != -1 && edge.s != -1)
            {
                facet_adjacency[edge.e].push_back(edge.s);
                facet_adjacency[edge.s].push_back(edge.e);
            }
        }

        for (uint32 i = 0; i < facet_count; ++i)
        {
            if (!visited_facets[i])
            {
                std::vector<uint32> component;
                std::vector<uint32> stack;
                stack.push_back(i);
                visited_facets[i] = true;

                while (!stack.empty())
                {
                    uint32 current = stack.back();
                    stack.pop_back();
                    component.push_back(current);

                    for (uint32 neighbor : facet_adjacency[current])
                    {
                        if (!visited_facets[neighbor])
                        {
                            visited_facets[neighbor] = true;
                            stack.push_back(neighbor);
                        }
                    }
                }
                connected_components.push_back(component);
            }
        }

        const uint32 chunk_count = connected_components.size();
        std::vector<NvBlastChunkDesc> chunk_descs(chunk_count);

        for (uint32 chunk_idx = 0; chunk_idx < chunk_count; ++chunk_idx)
        {
            const auto& component = connected_components[chunk_idx];

            float chunk_volume = 0.0f;
            NvcVec3 chunk_centroid = { 0, 0, 0 };
            float total_area = 0.0f;

            for (uint32 facet_idx : component)
            {
                const Facet& facet = facets[facet_idx];

                const Vertex& v0 = vertices[facet.firstEdgeNumber];
                const Vertex& v1 = vertices[edges[facet.firstEdgeNumber].s];
                const Vertex& v2 = vertices[edges[facet.firstEdgeNumber + 1].s];

                NvcVec3 edge1 = { v1.p.x - v0.p.x, v1.p.y - v0.p.y, v1.p.z - v0.p.z };
                NvcVec3 edge2 = { v2.p.x - v0.p.x, v2.p.y - v0.p.y, v2.p.z - v0.p.z };
                NvcVec3 cross = {
                    edge1.y * edge2.z - edge1.z * edge2.y,
                    edge1.z * edge2.x - edge1.x * edge2.z,
                    edge1.x * edge2.y - edge1.y * edge2.x
                };
                float area = std::sqrt(cross.x * cross.x + cross.y * cross.y + cross.z * cross.z) * 0.5f;

                NvcVec3 triCentroid = {
                    (v0.p.x + v1.p.x + v2.p.x) / 3.0f,
                    (v0.p.y + v1.p.y + v2.p.y) / 3.0f,
                    (v0.p.z + v1.p.z + v2.p.z) / 3.0f
                };

                chunk_centroid.x += triCentroid.x * area;
                chunk_centroid.y += triCentroid.y * area;
                chunk_centroid.z += triCentroid.z * area;
                total_area += area;

                chunk_volume += area * 0.01f;
            }

            if (total_area > 0.0f)
            {
                chunk_centroid.x /= total_area;
                chunk_centroid.y /= total_area;
                chunk_centroid.z /= total_area;
            }

            chunk_descs[chunk_idx].volume = chunk_volume > 0.0f ? chunk_volume : 1.0f / chunk_count;
            chunk_descs[chunk_idx].parentChunkDescIndex = (chunk_idx == 0) ? 0xFFFFFFFF : 0; // First chunk is root
            chunk_descs[chunk_idx].flags = (chunk_idx == 0) ? NvBlastChunkDesc::SupportFlag : 0;
            chunk_descs[chunk_idx].centroid[0] = chunk_centroid.x;
            chunk_descs[chunk_idx].centroid[1] = chunk_centroid.y;
            chunk_descs[chunk_idx].centroid[2] = chunk_centroid.z;
            chunk_descs[chunk_idx].userData = chunk_idx;
        }

        std::vector<NvBlastBondDesc> bond_descs;
        std::vector<std::vector<bool>> bond_exists(chunk_count, std::vector<bool>(chunk_count, false));

        for (uint32 edge_idx = 0; edge_idx < edge_count; ++edge_idx)
        {
            const Edge& edge = edges[edge_idx];
            if (edge.e != -1 && edge.s != -1)
            {
                uint32 chunk0 = find_chunk_for_facet(connected_components, edge.e);
                uint32 chunk1 = find_chunk_for_facet(connected_components, edge.s);

                if (chunk0 != chunk1 && !bond_exists[chunk0][chunk1])
                {
                    bond_exists[chunk0][chunk1] = true;
                    bond_exists[chunk1][chunk0] = true;

                    const Vertex& v0 = vertices[edge_idx];
                    const Vertex& v1 = vertices[edge.s];

                    const Facet& facet0 = facets[edge.e];
                    const Facet& facet1 = facets[edge.s];

                    NvcVec3 face_normal0 = calculate_face_normal(vertices, edges, facet0);
                    NvcVec3 face_normal1 = calculate_face_normal(vertices, edges, facet1);

                    NvcVec3 bond_normal = {
                        (face_normal0.x + face_normal1.x) * 0.5f,
                        (face_normal0.y + face_normal1.y) * 0.5f,
                        (face_normal0.z + face_normal1.z) * 0.5f
                    };

                    float length = std::sqrt(bond_normal.x * bond_normal.x + bond_normal.y * bond_normal.y + bond_normal.z * bond_normal.z);
                    if (length > 0.0f)
                    {
                        bond_normal.x /= length;
                        bond_normal.y /= length;
                        bond_normal.z /= length;
                    }

                    NvcVec3 edge_vector = {
                        v1.p.x - v0.p.x,
                        v1.p.y - v0.p.y,
                        v1.p.z - v0.p.z
                    };

                    float edge_length = std::sqrt(edge_vector.x * edge_vector.x + edge_vector.y * edge_vector.y + edge_vector.z * edge_vector.z);

                    NvBlastBondDesc bond{};
                    bond.chunkIndices[0] = chunk0;
                    bond.chunkIndices[1] = chunk1;
                    bond.bond.area = edge_length * 0.1f;

                    bond.bond.centroid[0] = (v0.p.x + v1.p.x) * 0.5f;
                    bond.bond.centroid[1] = (v0.p.y + v1.p.y) * 0.5f;
                    bond.bond.centroid[2] = (v0.p.z + v1.p.z) * 0.5f;

                    bond.bond.normal[0] = bond_normal.x;
                    bond.bond.normal[1] = bond_normal.y;
                    bond.bond.normal[2] = bond_normal.z;

                    bond_descs.push_back(bond);
                }
            }
        }

		NvBlastAssetDesc asset_desc{};
        asset_desc.chunkCount = chunk_count;
        asset_desc.chunkDescs = chunk_descs.data();
        asset_desc.bondCount = static_cast<uint32>(bond_descs.size());
        asset_desc.bondDescs = bond_descs.empty() ? nullptr : bond_descs.data();

        size_t asset_memory_size = NvBlastGetAssetMemorySize(&asset_desc, nullptr);
        size_t scratch_memory_size = NvBlastGetRequiredScratchForCreateAsset(&asset_desc, nullptr);

        void* asset_memory = _aligned_malloc(asset_memory_size, 16);
        void* scratch_memory = malloc(scratch_memory_size);

        if (asset_memory == nullptr || scratch_memory == nullptr)
		{
            if (asset_memory)
			{
				_aligned_free(asset_memory);
			}
            if (scratch_memory)
			{
				free(scratch_memory);
			}
            return nullptr;
        }

        NvBlastAsset* blast_asset = NvBlastCreateAsset(asset_memory, &asset_desc, scratch_memory, nullptr);

        free(scratch_memory);

        if (blast_asset == nullptr)
		{
            _aligned_free(asset_memory);
            return nullptr;
        }

        TkAsset* tk_asset = framework.createAsset(blast_asset);
        if (tk_asset == nullptr)
        {
            _aligned_free(asset_memory);
            free(blast_asset);
            return nullptr;
        }

		return tk_asset;
	}

}