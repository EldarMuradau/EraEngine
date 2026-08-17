#include "physics/destructions/destruction_system.h"
#include "physics/destructions/destructible_component.h"
#include "physics/core/physics.h"
#include "physics/core/physics_utils.h"
#include "physics/body_component.h"
#include "physics/destructions/destruction_utils.h"
#include "physics/shape_utils.h"
#include "physics/shape_component.h"
#include "physics/aggregate_holder_component.h"
#include "physics/scene_queries.h"
#include "physics/joint.h"

#include <core/cpu_profiling.h>
#include <core/traits.h>

#include <ecs/base_components/transform_component.h>
#include <ecs/update_groups.h>
#include <ecs/rendering/mesh_component.h>

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<DestructionSystem>("DestructionSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("physics")))
			.method("update", &DestructionSystem::update)(metadata("update_group", update_types::BEFORE_PHYSICS));
	}

	DestructionSystem::DestructionSystem(World* _world)
		: System(_world)
	{
	}

	void DestructionSystem::init()
	{
		entt::registry& registry = world->get_registry();
		registry.on_construct<DestructibleComponent>().connect<&DestructionSystem::on_destructible_created>(this);
	}

	void DestructionSystem::update(float dt)
	{
		ZoneScopedN("DestructionSystem::update");

		process_added_destructibles(dt);
	}

	void DestructionSystem::process_added_destructibles(float dt)
	{
		using namespace physx;

		ZoneScopedN("DestructionSystem::process_added_destructibles");

		ScopedSpinLock _lock{ sync };

		for (auto iter = destructibles_to_init.begin(); iter != destructibles_to_init.end();)
		{
			Entity entity = world->get_entity(*iter);

			DestructibleComponent* destructibe_component = entity.get_component<DestructibleComponent>();

			if (destructibe_component->load_state == DestructibleComponent::LoadState::UNLOADED)
			{
				destructibe_component->load_state = DestructibleComponent::LoadState::INITIALIZATION;
			}

			if (destructibe_component->base_type == DestructibleComponent::Type::FRACTURE_BASED)
			{
				switch (destructibe_component->load_state)
				{
				case DestructibleComponent::LoadState::INITIALIZATION:
				{
					if (!init_fracture_based_entity(entity, destructibe_component))
					{
						++iter;
						continue;
					}
					destructibe_component->load_state = DestructibleComponent::LoadState::POST_PROCESSING;
				}
				break;

				case DestructibleComponent::LoadState::POST_PROCESSING:
				{
					if (!connect_touching_chunks(entity, destructibe_component))
					{
						++iter;
						continue;
					}
					destructibe_component->load_state = DestructibleComponent::LoadState::LOADED;
				}
				break;

				default:
					break;
				}
				
			}
			else
			{
				// TODO: family based
			}

			if (destructibe_component->load_state == DestructibleComponent::LoadState::LOADED)
			{
				iter = destructibles_to_init.erase(iter);
			}
			else
			{
				++iter;
			}
		}
	}

	void DestructionSystem::on_destructible_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		destructibles_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}

	bool DestructionSystem::init_fracture_based_entity(Entity entity, DestructibleComponent* destructibe_component) const
	{
		MeshComponent* mesh_component = entity.get_component<MeshComponent>();

		ASSERT(!has_flag(mesh_component->mesh->flags, mesh_creation_flags_compact));
		ASSERT(destructibe_component->fracture_desc.chunks_count > 0);

		if (mesh_component->mesh->load_state != AssetLoadState::LOADED ||
			!mesh_component->mesh->model_asset.has_value())
		{
			return false;
		}

		ASSERT(!mesh_component->mesh->model_asset->meshes.empty());
		const MeshAsset& root_mesh = mesh_component->mesh->model_asset->meshes[0];

		ASSERT(!root_mesh.submeshes.empty());
		const SubmeshAsset& root_submesh = root_mesh.submeshes[0];

		std::vector<uint32> indices;
		indices.reserve(root_submesh.triangles.size() * 3);
		for (size_t i = 0; i < root_submesh.triangles.size(); ++i)
		{
			indices.push_back(root_submesh.triangles[i].a);
			indices.push_back(root_submesh.triangles[i].b);
			indices.push_back(root_submesh.triangles[i].c);
		}

		ref<NvMesh> nv_mesh = make_ref<NvMesh>(
			create_std_vector_px_vec3(root_submesh.positions),
			create_std_vector_px_vec3(root_submesh.normals),
			create_std_vector_px_vec2(root_submesh.uvs),
			indices
		);

		std::vector<std::pair<ref<SubmeshAsset>, ref<NvMesh>>> meshes;
		if (destructibe_component->fracture_desc.chunks_count == 1)
		{
			meshes.emplace_back(std::make_pair(nv_mesh->create_render_submesh(), nv_mesh));
		}
		else
		{
			meshes = FractureUtils::fracture_nvmesh_into_submeshes(destructibe_component->fracture_desc.chunks_count, nv_mesh);
		}

		std::vector<float> radius_array;
		radius_array.resize(destructibe_component->fracture_desc.chunks_count);

		destructibe_component->chunks = build_chunks(entity,
			entity.get_component<TransformComponent>()->get_world_transform(),
			get_default_pbr_material(),
			meshes,
			destructibe_component->fracture_desc.density,
			radius_array);

		for (EntityPtr chunk : destructibe_component->chunks)
		{
			DynamicBodyComponent* body_component = chunk.get().get_component<DynamicBodyComponent>();
			body_component->simulated.get_for_write() = true;
		}

		return true;
	}

	EntityPtr DestructionSystem::build_chunk(Entity parent,
		const trs& world_transform,
		const ref<pbr_material>& material,
		const std::pair<ref<SubmeshAsset>, ref<NvMesh>>& mesh, 
		float density,
		float& radius) const
	{
		mesh_builder builder{ mesh_creation_flags_default, mesh_index_uint32 };

		ref<MultiMesh> created_mesh = make_ref<MultiMesh>();

		builder.pushMesh(*mesh.first, 1.0f, &created_mesh->aabb);

		created_mesh->model_asset = ModelAsset();
		created_mesh->model_asset->meshes.emplace_back().submeshes.emplace_back(*mesh.first);
		created_mesh->model_asset->materials.emplace_back();
		created_mesh->model_asset->meshes[0].submeshes[0].material_index = 0;

		created_mesh->submeshes.push_back({ builder.endSubmesh(), {}, trs::identity, material });

		created_mesh->mesh = builder.createDXMesh();

		radius = created_mesh->aabb.volume();
		created_mesh->load_state = AssetLoadState::LOADED;

		Entity created_entity = world->create_entity("Chunk");
		created_entity.set_parent(parent.get_handle());
		created_entity.get_component<TransformComponent>()->set_world_transform(trs(world_transform.position, world_transform.rotation, vec3(1.0f)));
		created_entity.add_component<MeshComponent>(created_mesh);

		DestructibleFractureChunkComponent* chunk_component = created_entity.add_component<DestructibleFractureChunkComponent>();
		chunk_component->nv_mesh = mesh.second;

		ConvexMeshShapeComponent* convex_mesh_component = created_entity.add_component<ConvexMeshShapeComponent>();
		convex_mesh_component->asset = &created_mesh->model_asset->meshes[0];
		convex_mesh_component->collision_type = CollisionType::ALL;

		float mass = ShapeUtils::volume_of_mesh(*mesh.first) * density;
		ASSERT(mass > 0.0f);

		mass = max(1.0f, mass);

		DynamicBodyComponent* dynamic_body = created_entity.add_component<DynamicBodyComponent>();
		dynamic_body->mass.get_for_write() = mass;
		dynamic_body->use_gravity.get_for_write() = true;
		dynamic_body->max_depenetration_velocity.get_for_write() = 200.0f;
		dynamic_body->solver_position_iterations_count.get_for_write() = 64;
		if (PhysicsEngine::get_physics_core()->get_descriptor().enable_tgs_solver)
		{
			dynamic_body->solver_velocity_iterations_count.get_for_write() = 8;
		}
		else
		{
			dynamic_body->solver_velocity_iterations_count.get_for_write() = 32;
		}
		dynamic_body->simulated.get_for_write() = false;

		return EntityPtr(created_entity);
	}

	std::vector<EntityPtr> DestructionSystem::build_chunks(Entity parent,
		const trs& world_transform, 
		const ref<pbr_material>& material, 
		const std::vector<std::pair<ref<SubmeshAsset>, ref<NvMesh>>>& meshes,
		float density,
		std::vector<float>& radiuses) const
	{
		std::vector<EntityPtr> result;
		result.reserve(meshes.size());

		if (!PhysicsEngine::get_physics_core()->is_gpu())
		{
			AggregateHolderComponent* aggregate_holder = parent.add_component<AggregateHolderComponent>();
			aggregate_holder->enable_self_collision = true;
			aggregate_holder->max_actors = max(aggregate_holder->max_actors, static_cast<uint32>(meshes.size()));
		}

		for (size_t i = 0; i < meshes.size(); ++i)
		{
			result.emplace_back(build_chunk(parent, world_transform, material, meshes[i], density, radiuses[i]));
		}

		return result;
	}

	bool DestructionSystem::connect_touching_chunks(Entity parent, DestructibleComponent* destructibe_component) const
	{
		for (EntityPtr chunk_ptr : destructibe_component->chunks)
		{
			connect_with_neighbours(destructibe_component, chunk_ptr.get());
		}

		// TODO: achor chunks

		return true;
	}

	void DestructionSystem::connect_with_neighbours(DestructibleComponent* parent_destructibe_component, Entity chunk) const
	{
		const ConvexMeshShapeComponent* convex_mesh_component = chunk.get_component<ConvexMeshShapeComponent>();
		DestructibleFractureChunkComponent* chunk_component = chunk.get_component<DestructibleFractureChunkComponent>();

		physx::PxShape* shape = convex_mesh_component->get_shape();
		ASSERT(shape != nullptr);

		uint32 nb_vertices = 0;
		const physx::PxVec3* vertices = nullptr;

		physx::PxTransform shape_pose;

		PhysicsEngine::execute_read([&]()
			{
				const physx::PxGeometryHolder& shape_geometry = shape->getGeometry();
				const physx::PxConvexMeshGeometry& mesh_geometry = shape_geometry.convexMesh();

				const physx::PxConvexMesh* convex_mesh = mesh_geometry.convexMesh;

				nb_vertices = convex_mesh->getNbVertices();
				vertices = convex_mesh->getVertices();

				shape_pose = shape->getActor()->getGlobalPose();
				shape_pose = shape_pose * shape->getLocalPose();
			});

		OverlapQuery::Params params;
		params.geometry.type = SceneQueryGeometry::Type::SPHERE;
		params.geometry.sphere_radius = 0.01f;
		params.distance = 0.01f;

		constexpr const uint32 MAX_CONNECTED_TO_VERTEX_COUNT = 32;
		SceneQueryPositionedHit overlap_buffer[MAX_CONNECTED_TO_VERTEX_COUNT];

		for (uint32 vectex_id = 0; vectex_id < nb_vertices; ++vectex_id)
		{
			const physx::PxVec3& vertex = vertices[vectex_id];
			params.geometry_transform = physx::create_trs(shape_pose * physx::PxTransform(vertex));

			uint32 hits_count = 0;
			PhysicsEngine::execute_write([&]()
				{
					hits_count = OverlapQuery::all(world, params, overlap_buffer, MAX_CONNECTED_TO_VERTEX_COUNT);
				});

			if (hits_count == 0)
			{
				continue;
			}

			const trs inv_vertex_world_transform = invert(params.geometry_transform);

			for (uint32 hit_id = 0; hit_id < hits_count; ++hit_id)
			{
				if (overlap_buffer[hit_id].shape_component == nullptr)
				{
					continue;
				}

				Entity neighbour = overlap_buffer[hit_id].shape_component->get_entity();
				if (neighbour == chunk)
				{
					continue;
				}

				FixedJointComponent::BaseDescriptor descriptor;
				descriptor.connected_entity = chunk;
				descriptor.local_frame = inv_vertex_world_transform * chunk.get_component<TransformComponent>()->get_world_transform();
				descriptor.second_connected_entity = neighbour;
				descriptor.second_local_frame = inv_vertex_world_transform * neighbour.get_component<TransformComponent>()->get_world_transform();

				Entity connector = world->create_entity();
				FixedJointComponent* fixed_joint_component = connector.add_component<FixedJointComponent>(descriptor);
				fixed_joint_component->enable_collision.get_for_write() = true;

				const float chunk_mass = chunk.get_component<DynamicBodyComponent>()->mass;
				const float neighbour_mass = neighbour.get_component<DynamicBodyComponent>()->mass;

				fixed_joint_component->break_force.get_for_write() = parent_destructibe_component->fracture_desc.break_force * (chunk_mass + neighbour_mass);

				chunk_component->connectors.emplace_back(EntityPtr{ connector });
			}
		}
	}
}