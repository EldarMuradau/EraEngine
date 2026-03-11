#include "game/physics/physics_initializer.h"
#include "game/physics/gameplay_physics_types.h"

#include <ecs/update_groups.h>
#include <ecs/base_components/transform_component.h>

#include <physics/body_component.h>
#include <physics/shape_component.h>
#include <physics/basic_objects.h>
#include <physics/joint.h>
#include <physics/ragdolls/ragdoll_component.h>
#include <physics/collisions_holder_root_component.h>


namespace era_engine
{
	PhysicsInitializer::PhysicsInitializer(World* _world)
		: world(_world)
	{
	}

	PhysicsInitializer::~PhysicsInitializer()
	{
	}

	void PhysicsInitializer::init()
	{
		using namespace physics;

		CollisionsHolderRootComponent* collision_holder_rc = world->add_root_component<CollisionsHolderRootComponent>();
		ASSERT(collision_holder_rc != nullptr);

		collision_holder_rc->set_collision_filter(static_cast<uint32>(CollisionType::RAGDOLL), 
			static_cast<uint32>(CollisionType::TERRAIN), true);

		collision_holder_rc->set_collision_filter(static_cast<uint32>(CollisionType::RAGDOLL), 
			static_cast<uint32>(GameCollisionType::STATICS), true);

		collision_holder_rc->set_collision_filter(static_cast<uint32>(CollisionType::RAGDOLL),
			static_cast<uint32>(CollisionType::RAGDOLL), true);

		collision_holder_rc->set_collision_filter(static_cast<uint32>(CollisionType::RAGDOLL),
			static_cast<uint32>(GameCollisionType::DYNAMICS), true);

		collision_holder_rc->set_collision_filter(static_cast<uint32>(GameCollisionType::DYNAMICS),
			static_cast<uint32>(GameCollisionType::DYNAMICS), true);

		collision_holder_rc->set_collision_filter(static_cast<uint32>(GameCollisionType::STATICS),
			static_cast<uint32>(GameCollisionType::DYNAMICS), true);

		collision_holder_rc->set_collision_filter(static_cast<uint32>(CollisionType::TERRAIN),
			static_cast<uint32>(GameCollisionType::DYNAMICS), true);

		collision_holder_rc->set_collision_filter(static_cast<uint32>(CollisionType::TERRAIN),
			static_cast<uint32>(GameCollisionType::CCT), true);

		collision_holder_rc->set_collision_filter(static_cast<uint32>(CollisionType::VEHICLE),
			static_cast<uint32>(GameCollisionType::DYNAMICS), true);

		collision_holder_rc->set_collision_filter(static_cast<uint32>(CollisionType::VEHICLE),
			static_cast<uint32>(GameCollisionType::DYNAMICS), true);

		collision_holder_rc->set_collision_filter(static_cast<uint32>(CollisionType::VEHICLE),
			static_cast<uint32>(CollisionType::TERRAIN), true);

		collision_holder_rc->set_collision_filter(static_cast<uint32>(CollisionType::VEHICLE),
			static_cast<uint32>(CollisionType::RAGDOLL), true);

		collision_holder_rc->set_collision_filter(static_cast<uint32>(CollisionType::VEHICLE),
			static_cast<uint32>(GameCollisionType::CCT), true);
	}

}