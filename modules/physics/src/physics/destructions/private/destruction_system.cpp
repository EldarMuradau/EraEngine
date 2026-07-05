#include "physics/destructions/destruction_system.h"
#include "physics/destructions/destructible_component.h"
#include "physics/core/physics.h"
#include "physics/core/physics_utils.h"
#include "physics/body_component.h"

#include <core/cpu_profiling.h>

#include <ecs/base_components/transform_component.h>
#include <ecs/update_groups.h>
#include "ecs/rendering/mesh_component.h"

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

			PxRigidActor* rigid_actor = PhysicsUtils::get_body_component(entity)->get_rigid_actor();
			if (rigid_actor == nullptr)
			{
				++iter;
				continue;
			}

			if (destructibe_component->base_type == DestructibleComponent::Type::FRACTURE_BASED)
			{
				MeshComponent* mesh_component = entity.get_component<MeshComponent>();

				//mesh_component->mesh;

				//std::vector<uint32> indices;
				//for (size_t i = 0; i < meshAsset->triangles.size(); ++i)
				//{
				//	indices.push_back(meshAsset->triangles[i].a);
				//	indices.push_back(meshAsset->triangles[i].b);
				//	indices.push_back(meshAsset->triangles[i].c);
				//}

				//ref<NvMesh> nvMesh = make_ref<NvMesh>(
				//	createStdVectorPxVec3(meshAsset->positions),
				//	createStdVectorPxVec3(meshAsset->normals),
				//	createStdVectorPxVec2(meshAsset->uvs),
				//	indices
				//);
			}
			else
			{

			}

			iter = destructibles_to_init.erase(iter);
		}
	}

	void DestructionSystem::on_destructible_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		destructibles_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}
}