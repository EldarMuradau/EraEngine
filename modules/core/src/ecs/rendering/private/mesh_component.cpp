#include "ecs/rendering/mesh_component.h"
#include "ecs/world.h"

#include <rttr/registration>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		rttr::registration::class_<MeshComponent>("MeshComponent")
			.constructor<ref<Entity::EcsData>, ref<MultiMesh>, bool>()
			.property("mesh", &MeshComponent::mesh)
			.property("is_hidden", &MeshComponent::is_hidden);
	}

	MeshComponent::MeshComponent(ref<Entity::EcsData> _data, ref<MultiMesh> _mesh, bool _is_hidden)
		: Component(_data), mesh(_mesh), is_hidden(_is_hidden)
	{
	}

	MeshComponent::~MeshComponent()
	{
	}

	Entity MeshUtils::load_entity_mesh_from_file(ref<World> world, const fs::path& filename, uint32 flags, MeshLoadCallback cb)
	{
		ref<MultiMesh> mesh = import_mesh_from_file(filename, flags, cb);
		Entity entity = world->create_entity();
		entity.add_component<MeshComponent>(mesh, false);
		return entity;
	}

	Entity MeshUtils::load_entity_mesh_from_handle(ref<World> world, AssetHandle handle, uint32 flags, MeshLoadCallback cb)
	{
		ref<MultiMesh> mesh = import_mesh_from_handle(handle, flags, cb);
		Entity entity = world->create_entity();
		entity.add_component<MeshComponent>(mesh, false);
		return entity;
	}

	Entity MeshUtils::load_entity_mesh_from_file_async(ref<World> world, const fs::path& filename, uint32 flags, MeshLoadCallback cb, JobHandle parent_job)
	{
		ref<MultiMesh> mesh = import_mesh_from_file_async(filename, flags, parent_job, cb);
		Entity entity = world->create_entity();
		entity.add_component<MeshComponent>(mesh, false);
		return entity;
	}

	Entity MeshUtils::load_entity_mesh_from_handle_async(ref<World> world, AssetHandle handle, uint32 flags, MeshLoadCallback cb, JobHandle parent_job)
	{
		ref<MultiMesh> mesh = import_mesh_from_handle_async(handle, flags, parent_job, cb);
		Entity entity = world->create_entity();
		entity.add_component<MeshComponent>(mesh, false);
		return entity;
	}

}