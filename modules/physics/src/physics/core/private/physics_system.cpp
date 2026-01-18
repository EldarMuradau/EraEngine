#include "physics/core/physics_system.h"
#include "physics/core/physics.h"
#include "physics/aggregate_holder_component.h"
#include "physics/body_component.h"
#include "physics/shape_component.h"
#include "physics/core/physics_utils.h"
#include "physics/shape_utils.h"
#include "physics/aggregate.h"
#include "physics/core/physics_types.h"
#include "physics/cct_component.h"

#include <core/cpu_profiling.h>

#include <ecs/base_components/transform_component.h>
#include <ecs/update_groups.h>

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<PhysicsSystem>("PhysicsSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("physics")))
			.method("clear_pending_collisions", &PhysicsSystem::clear_pending_collisions)(metadata("update_group", update_types::GAMEPLAY_BEFORE_PHYSICS_CONCURRENT))
			.method("update", &PhysicsSystem::update)(metadata("update_group", update_types::PHYSICS));
	}

	PhysicsSystem::PhysicsSystem(World* _world)
		: System(_world)
	{
	}

	void PhysicsSystem::init()
	{
		cct_hit_report_callback = make_ref<CharacterControllerHitReportCallback>(world);

		entt::registry& registry = world->get_registry();
		registry.on_construct<DynamicBodyComponent>().connect<&PhysicsSystem::on_dynamic_body_created>(this);
		registry.on_construct<StaticBodyComponent>().connect<&PhysicsSystem::on_static_body_created>(this);
		registry.on_construct<AggregateHolderComponent>().connect<&PhysicsSystem::on_aggregate_created>(this);
		registry.on_construct<CharacterControllerComponent>().connect<&PhysicsSystem::on_cct_created>(this);

		dynamic_body_group = world->group(components_group<TransformComponent, DynamicBodyComponent>);
	}

	void PhysicsSystem::update(float dt)
	{
		ZoneScopedN("PhysicsSystem::update");

		{
			ZoneScopedN("PhysicsSystem::process_added_bodies");

			process_added_bodies();
		}

		{
			ZoneScopedN("PhysicsSystem::process_added_ccts");

			process_added_ccts();
		}

		{
			ZoneScopedN("PhysicsSystem::sync_physics_to_component_changes");

			sync_physics_to_component_changes(dt);
		}

		{
			ZoneScopedN("PhysicsSystem::update PhysX");

			ScopedSpinLock _lock{ PhysicsHolder::physics_ref->sync };

			PhysicsHolder::physics_ref->update(dt);
		}

		{
			ZoneScopedN("PhysicsSystem::sync_component_to_physics");

			sync_component_to_physics();
		}
	}

	void PhysicsSystem::clear_pending_collisions(float)
	{
		ZoneScopedN("PhysicsSystem::clear_pending_collisions");

		ScopedSpinLock _lock{ PhysicsHolder::physics_ref->sync };

		PhysicsHolder::physics_ref->clear_collisions();
	}

	void PhysicsSystem::sync_physics_to_component_changes(float dt)
	{
		using namespace physx;

		PxSceneWriteLock _scene_lock{ *PhysicsHolder::physics_ref->get_scene() };

		for (auto [entity_handle, changed_flag, static_body] : world->group(components_group<TransformComponent, StaticBodyComponent>).each())
		{
			if (static_body.get_rigid_actor() == nullptr)
			{
				continue;
			}

			PxRigidStatic* body = static_body.get_rigid_static();

			if (body == nullptr)
			{
				continue;
			}

			if (static_body.simulated.is_changed())
			{
				body->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, !static_body.simulated);
				static_body.simulated.sync_changes();
			}
		}

		for (auto [entity_handle, changed_flag, dynamic_body] : world->group(components_group<TransformComponent, DynamicBodyComponent>).each())
		{
			if (dynamic_body.get_rigid_actor() == nullptr)
			{
				continue;
			}

			PxRigidDynamic* body = dynamic_body.get_rigid_dynamic();

			if (body == nullptr)
			{
				continue;
			}

			if (dynamic_body.max_linear_velocity.is_changed())
			{
				body->setMaxLinearVelocity(dynamic_body.max_linear_velocity);
				dynamic_body.max_linear_velocity.sync_changes();
			}

			if (dynamic_body.max_depenetration_velocity.is_changed())
			{
				body->setMaxDepenetrationVelocity(dynamic_body.max_depenetration_velocity);
				dynamic_body.max_contact_impulse.sync_changes();
			}

			if (dynamic_body.max_contact_impulse.is_changed())
			{
				body->setMaxContactImpulse(dynamic_body.max_contact_impulse);
				dynamic_body.max_contact_impulse.sync_changes();
			}

			if (dynamic_body.max_angular_velocity.is_changed())
			{
				body->setMaxAngularVelocity(dynamic_body.max_angular_velocity);
				dynamic_body.max_angular_velocity.sync_changes();
			}

			bool update_mass = false;
			bool only_started_simulation = false;

			if (dynamic_body.mass.is_changed() ||
				dynamic_body.center_of_mass.is_changed() ||
				dynamic_body.mass_space_inertia_tensor.is_changed())
			{
				update_mass = true;

				dynamic_body.mass_space_inertia_tensor.sync_changes();
				dynamic_body.center_of_mass.sync_changes();
				dynamic_body.mass.sync_changes();
			}

			if (dynamic_body.simulated.is_changed())
			{
				body->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, !dynamic_body.simulated);

				if (dynamic_body.simulated)
				{
					update_mass = true;
					only_started_simulation = true;
				}

				dynamic_body.simulated.sync_changes();
			}

			if (update_mass)
			{
				PhysicsUtils::manual_update_mass(&dynamic_body);
			}

			if (dynamic_body.use_gravity.is_changed())
			{
				body->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, !dynamic_body.use_gravity);
				dynamic_body.use_gravity.sync_changes();
			}

			if (dynamic_body.solver_position_iterations_count.is_changed() ||
				dynamic_body.solver_velocity_iterations_count.is_changed())
			{
				body->setSolverIterationCounts(dynamic_body.solver_position_iterations_count, dynamic_body.solver_velocity_iterations_count);
				dynamic_body.solver_position_iterations_count.sync_changes();
				dynamic_body.solver_velocity_iterations_count.sync_changes();
			}

			if (dynamic_body.sleep_threshold.is_changed())
			{
				body->setSleepThreshold(dynamic_body.sleep_threshold);
				dynamic_body.sleep_threshold.sync_changes();
			}

			if (dynamic_body.stabilization_threshold.is_changed())
			{
				body->setStabilizationThreshold(dynamic_body.stabilization_threshold);
				dynamic_body.stabilization_threshold.sync_changes();
			}

			if (dynamic_body.ccd.is_changed())
			{
				body->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, dynamic_body.ccd);
				dynamic_body.ccd.sync_changes();
			}

			if (dynamic_body.kinematic.is_changed())
			{
				body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, dynamic_body.kinematic);
				dynamic_body.kinematic.sync_changes();
			}

			if (only_started_simulation && !dynamic_body.kinematic)
			{
				body->setLinearVelocity(create_PxVec3(dynamic_body.linear_velocity), false);
				body->setAngularVelocity(create_PxVec3(dynamic_body.angular_velocity), false);
			}

			if (dynamic_body.constraints.is_changed())
			{
				body->setRigidDynamicLockFlags(static_cast<PxRigidDynamicLockFlags>(dynamic_body.constraints));
				dynamic_body.constraints.sync_changes();
			}

			if (dynamic_body.linear_damping.is_changed())
			{
				body->setLinearDamping(dynamic_body.linear_damping);
				dynamic_body.linear_damping.sync_changes();
			}

			if (dynamic_body.angular_damping.is_changed())
			{
				body->setAngularDamping(dynamic_body.angular_damping);
				dynamic_body.angular_damping.sync_changes();
			}

			if (dynamic_body.angular_velocity.is_changed())
			{
				if (!dynamic_body.kinematic && dynamic_body.simulated)
				{
					body->setAngularVelocity(create_PxVec3(dynamic_body.angular_velocity));
				}
				dynamic_body.angular_velocity.sync_changes();
			}

			if (dynamic_body.linear_velocity.is_changed())
			{
				if(!dynamic_body.kinematic && dynamic_body.simulated)
				{
					body->setLinearVelocity(create_PxVec3(dynamic_body.linear_velocity));
				}
				dynamic_body.linear_velocity.sync_changes();
			}

			for (const Force& force : std::exchange(dynamic_body.forces, {}))
			{
				if (fuzzy_equals(force.force, vec3::zero))
				{
					continue;
				}

				vec3 value = force.force;
				if (force.mode == ForceMode::IMPULSE)
				{
					const float max_contact_impulse = body->getMaxContactImpulse();
					const float value_magnitude = length(value);

					value = normalize(value) * std::min(max_contact_impulse, value_magnitude);
				}

				if (force.mode == ForceMode::FORCE || force.mode == ForceMode::IMPULSE)
				{
					PxRigidBodyExt::addForceAtLocalPos(*body,
						create_PxVec3(value),
						create_PxVec3(force.point),
						static_cast<PxForceMode::Enum>(force.mode));
				}
				else
				{
					body->addForce(create_PxVec3(force.force), static_cast<PxForceMode::Enum>(force.mode));
				}
			}

			for (const Torque& torque : std::exchange(dynamic_body.torques, {}))
			{
				if (fuzzy_equals(torque.torque, vec3::zero))
				{
					continue;
				}

				vec3 value = torque.torque;
				if (torque.mode == ForceMode::IMPULSE)
				{
					const float max_contact_impulse = body->getMaxContactImpulse();
					const float value_magnitude = length(value);

					value = normalize(value) * std::min(max_contact_impulse, value_magnitude);
				}

				body->addTorque(create_PxVec3(value),
					static_cast<PxForceMode::Enum>(torque.mode));
			}
		}

		for (auto [entity_handle, changed_flag, cct_component] : world->group(components_group<TransformComponent, CharacterControllerComponent>).each())
		{
			PxCapsuleController* controller = cct_component.controller;

			if (controller == nullptr)
			{
				continue;
			}

			const vec3 offset = cct_component.velocity.get() * dt + cct_component.offset.get();

			if (!fuzzy_equals(offset, vec3::zero))
			{
				PhysicsUtils::move_cct(&cct_component, offset);
			}

			cct_component.offset = vec3::zero;
		}
	}

	void PhysicsSystem::sync_component_to_physics()
	{
		using namespace physx;

		for (auto [entity_handle, transformm_component, dynamic_body] : dynamic_body_group.each())
		{
			PxRigidDynamic* body = dynamic_body.get_rigid_dynamic();

			if (body == nullptr)
			{
				return;
			}

			if (!dynamic_body.kinematic)
			{
				dynamic_body.linear_velocity.get_silent_for_write() = create_vec3(body->getLinearVelocity());
				dynamic_body.angular_velocity.get_silent_for_write() = create_vec3(body->getAngularVelocity());
			}
			else
			{
				dynamic_body.linear_velocity.get_silent_for_write() = vec3::zero;
				dynamic_body.angular_velocity.get_silent_for_write() = vec3::zero;
			}
		}
	}

	void PhysicsSystem::process_added_bodies()
	{
		using namespace physx;

		ScopedSpinLock _lock{ PhysicsHolder::physics_ref->sync };

		auto& physics_ref = PhysicsHolder::physics_ref;
		const auto& physics = physics_ref->get_physics();

		PxSceneWriteLock _scene_lock { *physics_ref->get_scene()};

		for (Entity::Handle entity_handle : std::exchange(aggregates_to_init, {}))
		{
			Entity entity = world->get_entity(entity_handle);

			AggregateHolderComponent* aggregate_holder_component = entity.get_component<AggregateHolderComponent>();
			aggregate_holder_component->aggregate = make_ref<Aggregate>(aggregate_holder_component->max_actors, aggregate_holder_component->enable_self_collision);
		}

		for (Entity::Handle entity_handle : std::exchange(dynamics_to_init, {}))
		{
			Entity entity = world->get_entity(entity_handle);

			DynamicBodyComponent* dynamic_body_component = entity.get_component<DynamicBodyComponent>();

			auto& colliders = physics_ref->colliders_map[entity_handle];

			TransformComponent* transform = entity.get_component_if_exists<TransformComponent>();
			transform->type = TransformComponent::DYNAMIC;

			const trs& world_transform = transform->get_world_transform();

			PxVec3 pospx = create_PxVec3(world_transform.position);

			PxQuat rotpx = create_PxQuat(world_transform.rotation);

			void* user_data = static_cast<void*>(dynamic_body_component);

			dynamic_body_component->actor = PhysicsUtils::create_rigid_dynamic(PxTransform(pospx, rotpx), user_data);

			for (ShapeComponent* shape_component : colliders)
			{
				PxShape* shape = shape_component->create_shape();

				dynamic_body_component->actor->attachShape(*shape);

				ShapeUtils::setup_filtering(world, shape, static_cast<uint32>(shape_component->collision_type.get()), shape_component->collision_filter_data);
			}

			physics_ref->add_actor(dynamic_body_component, dynamic_body_component->actor);

			{
				PxRigidDynamic* dynamic_body = dynamic_body_component->get_rigid_dynamic();

				const PxVec3 center_of_mass = create_PxVec3(dynamic_body_component->center_of_mass);

				PxRigidBodyExt::setMassAndUpdateInertia(
					*dynamic_body,
					dynamic_body_component->mass.get(),
					&center_of_mass,
					false);

				dynamic_body_component->center_of_mass.get_silent_for_write() = create_vec3(dynamic_body->getCMassLocalPose().p);
			}
		}

		for (Entity::Handle entity_handle : std::exchange(statics_to_init, {}))
		{
			Entity entity = world->get_entity(entity_handle);

			StaticBodyComponent* static_body_component = entity.get_component<StaticBodyComponent>();

			auto& colliders = physics_ref->colliders_map[entity_handle];

			const TransformComponent* transform = entity.get_component_if_exists<TransformComponent>();

			const trs& world_transform = transform->get_world_transform();

			PxVec3 pospx = create_PxVec3(world_transform.position);

			PxQuat rotpx = create_PxQuat(world_transform.rotation);

			void* user_data = static_cast<void*>(static_body_component);

			static_body_component->actor = PhysicsUtils::create_rigid_static(PxTransform(pospx, rotpx), user_data);

			for (ShapeComponent* shape_component : colliders)
			{
				PxShape* shape = shape_component->create_shape();

				static_body_component->actor->attachShape(*shape);
				ShapeUtils::setup_filtering(world, shape, static_cast<uint32>(shape_component->collision_type.get()), shape_component->collision_filter_data);
			}

			physics_ref->add_actor(static_body_component, static_body_component->actor);
		}
	}

	void PhysicsSystem::process_added_ccts()
	{
		using namespace physx;

		for (Entity::Handle entity_handle : std::exchange(ccts_to_init, {}))
		{
			Entity entity = world->get_entity(entity_handle);

			CharacterControllerComponent* cct_component = entity.get_component<CharacterControllerComponent>();

			const TransformComponent* transform = entity.get_component_if_exists<TransformComponent>();

			const trs& world_transform = transform->get_world_transform();

			PxExtendedVec3 pospx = create_PxExtendedVec3(world_transform.position + vec3(0.0f, cct_component->radius + cct_component->height / 2.0f, 0.0f));

			void* user_data = static_cast<void*>(cct_component);

			ASSERT(cct_component->radius > 0.0f);
			ASSERT(cct_component->height > 0.0f);

			ref<PhysicsMaterial> material = PhysicsHolder::physics_ref->create_material(0.5f, 1.0f, 0.1f);
			ASSERT(material != nullptr);

			PxCapsuleControllerDesc desc;
			desc.radius = cct_component->radius / 0.99f;
			desc.height = cct_component->height / 0.99f;
			desc.material = material->get_native_material();
			desc.scaleCoeff = 0.99f;
			desc.slopeLimit = std::cosf(cct_component->slope_limit);
			desc.reportCallback = static_cast<CharacterControllerHitReportCallback*>(cct_hit_report_callback.get());
			desc.contactOffset = cct_component->contact_offset;
			desc.volumeGrowth = cct_component->volume_growth;
			desc.position = pospx;

			if (cct_component->non_walkable_mode == CharacterControllerComponent::NonWalkableMode::PREVENT_CLIMBING)
			{
				desc.nonWalkableMode = PxControllerNonWalkableMode::ePREVENT_CLIMBING;
			}
			else
			{
				ASSERT(cct_component->non_walkable_mode == CharacterControllerComponent::NonWalkableMode::PREVENT_CLIMBING_AND_SLIDE);
				desc.nonWalkableMode = PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING;
			}

			cct_component->controller = static_cast<PxCapsuleController*>(PhysicsHolder::physics_ref->cct_manager->createController(desc));

			cct_component->controller->setUserData(user_data);

			PxRigidDynamic* physx_actor = cct_component->controller->getActor();
			ASSERT(physx_actor != nullptr);
			physx_actor->userData = user_data;

			PxShape* physx_shape = nullptr;
			physx_actor->getShapes(&physx_shape, 1, 0);
			ASSERT(physx_shape != nullptr);
			physx_shape->userData = user_data;

			ShapeUtils::setup_filtering(world, physx_shape, static_cast<uint32>(cct_component->collision_type.get()), cct_component->collision_filter_data);
		}
	}

	void PhysicsSystem::on_dynamic_body_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ PhysicsHolder::physics_ref->sync };

		dynamics_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}

	void PhysicsSystem::on_static_body_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ PhysicsHolder::physics_ref->sync };

		statics_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}

	void PhysicsSystem::on_aggregate_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ PhysicsHolder::physics_ref->sync };

		aggregates_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}

	void PhysicsSystem::on_cct_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ PhysicsHolder::physics_ref->sync };

		ccts_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}

}
