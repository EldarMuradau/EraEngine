#include "physics/pbd/private/pbd_cloth_system.h"
#include "physics/core/physics.h"
#include "physics/core/physics_utils.h"
#include "physics/body_component.h"
#include "physics/pbd/pbd_cloth_component.h"

#include <core/cpu_profiling.h>

#include <ecs/base_components/transform_component.h>
#include <ecs/rendering/cloth_render_component.h>
#include <ecs/update_groups.h>

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<PBDClothSystem>("PBDClothSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("physics")))
			.method("update", &PBDClothSystem::update)(metadata("update_group", update_types::BEFORE_PHYSICS));
	}

	PBDClothSystem::PBDClothSystem(World* _world)
		: System(_world)
	{
	}

	void PBDClothSystem::init()
	{
		entt::registry& registry = world->get_registry();
		registry.on_construct<PBDClothComponent>().connect<&PBDClothSystem::on_cloth_created>(this);
	}

	void PBDClothSystem::update(float dt)
	{
		ZoneScopedN("PBDClothSystem::update");

		process_added_clothes(dt);

		for (auto&& [entity_handle, transform_component, cloth_component] : world->group(components_group<TransformComponent, PBDClothComponent>).each())
		{

		}
	}

	void PBDClothSystem::process_added_clothes(float dt)
	{
		using namespace physx;

		ZoneScopedN("PBDClothSystem::process_added_clothes");

		ScopedSpinLock _lock{ sync };

		for (auto iter = clothes_to_init.begin(); iter != clothes_to_init.end();)
		{
			Entity entity = world->get_entity(*iter);

			PBDClothComponent* cloth_component = entity.get_component<PBDClothComponent>();

			ClothRenderComponent* render_component = entity.add_component<ClothRenderComponent>();
			render_component->get_data_internal = std::bind(&PBDClothComponent::get_render_data, cloth_component, std::placeholders::_1);

			PxRigidActor* rigid_actor = PhysicsUtils::get_body_component(entity)->get_rigid_actor();
			if (rigid_actor == nullptr)
			{
				++iter;
				continue;
			}

			iter = clothes_to_init.erase(iter);
		}
	}

	void PBDClothSystem::on_cloth_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		clothes_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}
}