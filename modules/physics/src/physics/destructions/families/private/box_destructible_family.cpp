#include "physics/destructions/families/box_destructible_family.h"
#include "physics/physx_api.h"
#include "physics/core/physics.h"
#include "physics/destructions/blast_physx/NvBlastExtPxManager.h"
#include "physics/destructions/blast_physx/NvBlastExtPxAsset.h"
#include "physics/destructions/blast_physx/NvBlastExtPxActor.h"
#include "physics/destructions/blast_physx/NvBlastExtPxFamily.h"

#include <ecs/world.h>
#include <ecs/base_components/transform_component.h>
#include <ecs/rendering/mesh_component.h>

namespace era_engine::physics
{
	BoxDestructibleFamily::BoxDestructibleFamily(World* _world,
		Nv::Blast::ExtPxManager& _px_manager,
		DestructibleAsset* _blast_asset,
		const DestructibleAsset::ActorDesc& desc)
		: DestructibleFamily(_world, _px_manager, _blast_asset)
	{
		using namespace Nv::Blast;

		mesh_builder builder;

		box_render_material = create_pbr_material_async({ "", "" });
		box_render_material->shader = pbr_material_shader_double_sided;

		box_mesh = make_ref<MultiMesh>();
		builder.pushBox({ });
		box_mesh->submeshes.push_back({ builder.endSubmesh(), {}, trs::identity, box_render_material });

		box_mesh->mesh = builder.createDXMesh();

		const ExtPxAsset* px_asset = blast_asset->px_asset;
		const uint32 chunk_count = px_asset->getChunkCount();
		const ExtPxChunk* chunks = px_asset->getChunks();
		const ExtPxSubchunk* sub_chunks = px_asset->getSubchunks();

		const Entity::Handle root_handle = root_entity.get().get_handle();

		entities.reserve(chunk_count);
		for (uint32 i = 0; i < chunk_count; ++i)
		{
			Entity chunk_entity = world->create_entity();

			chunk_entity.set_parent(root_handle);

			const vec3 scale = physx::create_vec3(sub_chunks[chunks[i].firstSubchunkIndex].geometry.scale.scale);

			TransformComponent* transform_component = chunk_entity.get_component<TransformComponent>();
			transform_component->set_world_scale(scale);

			MeshComponent* mesh_component = chunk_entity.add_component<MeshComponent>(box_mesh);
			mesh_component->is_hidden = true;

			entities.emplace_back(chunk_entity);
		}

		initialize(desc);
	}

	BoxDestructibleFamily::~BoxDestructibleFamily()
	{
	}

	void BoxDestructibleFamily::on_actor_created(const Nv::Blast::ExtPxActor& actor)
	{
		const uint32* chunk_indices = actor.getChunkIndices();
		uint32 chunk_count = actor.getChunkCount();
		for (uint32 i = 0; i < chunk_count; ++i)
		{
			const uint32 chunk_index = chunk_indices[i];

			Entity chunk_entity = entities[chunk_index].get();

			chunk_entity.get_component<MeshComponent>()->is_hidden = false;
		}
	}

	void BoxDestructibleFamily::on_actor_update(const Nv::Blast::ExtPxActor& actor)
	{
		using namespace physx;
		using namespace Nv::Blast;

		const ExtPxChunk* chunks = blast_asset->px_asset->getChunks();
		const ExtPxSubchunk* sub_chunks = blast_asset->px_asset->getSubchunks();
		const uint32* chunk_indices = actor.getChunkIndices();
		uint32 chunk_count = actor.getChunkCount();
		for (uint32 i = 0; i < chunk_count; ++i)
		{
			const uint32 chunk_index = chunk_indices[i];

			const PxTransform world_transform = actor.getPhysXActor().getGlobalPose() * sub_chunks[chunks[chunk_index].firstSubchunkIndex].transform;

			Entity chunk_entity = entities[chunk_index].get();
			TransformComponent* transform = chunk_entity.get_component<TransformComponent>();

			transform->set_world_transform(create_trs(world_transform));
		}
	}

	void BoxDestructibleFamily::on_actor_destroyed(const Nv::Blast::ExtPxActor& actor)
	{
		const uint32* chunk_indices = actor.getChunkIndices();
		uint32 chunk_count = actor.getChunkCount();
		for (uint32 i = 0; i < chunk_count; ++i)
		{
			const uint32 chunk_index = chunk_indices[i];

			Entity chunk_entity = entities[chunk_index].get();

			chunk_entity.get_component<MeshComponent>()->is_hidden = true;
		}
	}
}