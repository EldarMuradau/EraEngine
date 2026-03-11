#include "physics/vehicles/vehicle_system.h"
#include "physics/core/physics.h"
#include "physics/snippets_utils/serialization/BaseSerialization.h"
#include "physics/snippets_utils/serialization/EngineDrivetrainSerialization.h"

#include <ecs/update_groups.h>
#include <ecs/base_components/name_component.h>

#include <core/log.h>
#include <core/string.h>

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<VehicleSystem>("VehicleSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("physics")))

			.method("update", &VehicleSystem::update)
			(metadata("update_group", update_types::PHYSICS),
			 metadata("Before", std::vector<std::string>{"PhysicsSystem::update"}));
	}

	VehicleSystem::VehicleSystem(World* _world)
		: System(_world)
	{
	}

	VehicleSystem::~VehicleSystem()
	{
	}

	void VehicleSystem::init()
	{
		entt::registry& registry = world->get_registry();
		registry.on_construct<W4VehicleComponent>().connect<&VehicleSystem::on_vehicle_created>(this);
		registry.on_destroy<W4VehicleComponent>().connect<&VehicleSystem::on_vehicle_released>(this);

		w4_vehicles_group = world->group(components_group<TransformComponent, W4VehicleComponent>);
		tank_vehicles_group = world->group(components_group<TransformComponent, TankVehicleComponent>);
	}

	void VehicleSystem::update(float dt)
	{
		using namespace physx;

		process_added_vehicles();

		for (auto [entity_handle, transformm_component, w4_vehicle_component] : w4_vehicles_group.each())
		{
			const PxVec3 lin_vel = w4_vehicle_component.vehicle->mPhysXState.physxActor.rigidBody->getLinearVelocity();
			const PxVec3 forward_dir = w4_vehicle_component.vehicle->mPhysXState.physxActor.rigidBody->getGlobalPose().q.getBasisVector2();
			const PxReal forward_speed = lin_vel.dot(forward_dir);
			const PxU8 nb_substeps = (forward_speed < 5.0f ? 3 : 1);
			w4_vehicle_component.vehicle->mComponentSequence.setSubsteps(w4_vehicle_component.vehicle->mComponentSequenceSubstepGroupHandle, nb_substeps);
			w4_vehicle_component.vehicle->step(dt, *w4_vehicle_component.vehicle_simulation_context);
		}

		for (auto [entity_handle, transformm_component, tank_vehicle_component] : tank_vehicles_group.each())
		{
			const PxVec3 lin_vel = w4_vehicle_component.vehicle->mPhysXState.physxActor.rigidBody->getLinearVelocity();
			const PxVec3 forward_dir = w4_vehicle_component.vehicle->mPhysXState.physxActor.rigidBody->getGlobalPose().q.getBasisVector2();
			const PxReal forward_speed = lin_vel.dot(forward_dir);
			const PxU8 nb_substeps = (forward_speed < 5.0f ? 3 : 1);
			w4_vehicle_component.vehicle->mComponentSequence.setSubsteps(w4_vehicle_component.vehicle->mComponentSequenceSubstepGroupHandle, nb_substeps);
			w4_vehicle_component.vehicle->step(dt, *w4_vehicle_component.vehicle_simulation_context);
		}
	}

	void VehicleSystem::process_added_vehicles()
	{
		using namespace physx;
		using namespace physx::vehicle2;
		using namespace snippetvehicle2;

		ScopedSpinLock _lock{ sync };

		const auto& physics = PhysicsHolder::physics_ref;

		for (Entity::Handle entity_handle : std::exchange(vehicles_to_init, {}))
		{
			Entity entity = world->get_entity(entity_handle);

			if (W4VehicleComponent* w4_vehicle_component = entity.get_component_if_exists<W4VehicleComponent>())
			{
				w4_vehicle_component->vehicle = new EngineDriveVehicle();
				w4_vehicle_component->vehicle_simulation_context = new PxVehiclePhysXSimulationContext();

				std::string path = get_asset_path("/resources/assets/vehicledata");
				BaseVehicleParams base_params;
				if (!readBaseParamsFromJsonFile(path.c_str(), "Base.json", base_params))
				{
					LOG_ERROR("Failed to create vehicle");
					continue;
				}

				EngineDrivetrainParams engine_drivetrain_params;
				if (!readEngineDrivetrainParamsFromJsonFile(path.c_str(),
					"EngineDrive.json",
					engine_drivetrain_params))
				{
					LOG_ERROR("Failed to create vehicle");
					continue;
				}

				readBaseParamsFromJsonFile(path.c_str(), "Base.json", w4_vehicle_component->vehicle->mBaseParams);
				setPhysXIntegrationParams(w4_vehicle_component->vehicle->mBaseParams.axleDescription,
					w4_vehicle_component->material_frictions, 
					w4_vehicle_component->nb_material_frictions, 
					w4_vehicle_component->default_material_friction,
					w4_vehicle_component->vehicle->mPhysXParams);

				readEngineDrivetrainParamsFromJsonFile(path.c_str(), "EngineDrive.json",
					w4_vehicle_component->vehicle->mEngineDriveParams);

				if (!w4_vehicle_component->vehicle->initialize(*physics->get_physics(), PxCookingParams(physics->get_tolerance_scale()), *w4_vehicle_component->material->get_native_material(), EngineDriveVehicle::eDIFFTYPE_FOURWHEELDRIVE))
				{
					LOG_ERROR("Failed to create vehicle");
					continue;
				}

				PxTransform pose = create_PxTransform(entity.get_component<TransformComponent>()->get_world_transform());
				w4_vehicle_component->vehicle->setUpActor(world, 
					*physics->get_scene(), 
					pose, 
					entity.get_component<NameComponent>()->name, 
					static_cast<uint32>(w4_vehicle_component->collision_type));

				void* user_data = static_cast<void*>(w4_vehicle_component);
				w4_vehicle_component->vehicle->mPhysXState.physxActor.rigidBody->userData = user_data;

				w4_vehicle_component->vehicle->mEngineDriveState.gearboxState.currentGear = w4_vehicle_component->vehicle->mEngineDriveParams.gearBoxParams.neutralGear + 1;
				w4_vehicle_component->vehicle->mEngineDriveState.gearboxState.targetGear = w4_vehicle_component->vehicle->mEngineDriveParams.gearBoxParams.neutralGear + 1;

				w4_vehicle_component->vehicle->mTransmissionCommandState.targetGear = PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR;

				w4_vehicle_component->vehicle_simulation_context->setToDefault();
				w4_vehicle_component->vehicle_simulation_context->frame.lngAxis = PxVehicleAxes::ePosZ;
				w4_vehicle_component->vehicle_simulation_context->frame.latAxis = PxVehicleAxes::ePosX;
				w4_vehicle_component->vehicle_simulation_context->frame.vrtAxis = PxVehicleAxes::ePosY;
				w4_vehicle_component->vehicle_simulation_context->scale.scale = 1.0f;
				w4_vehicle_component->vehicle_simulation_context->gravity = gravity;
				w4_vehicle_component->vehicle_simulation_context->physxScene = physics->get_scene();
				w4_vehicle_component->vehicle_simulation_context->physxActorUpdateMode = PxVehiclePhysXActorUpdateMode::eAPPLY_ACCELERATION;
			}
			else if (TankVehicleComponent* tank_vehicle_component = entity.get_component_if_exists<TankVehicleComponent>())
			{
				LOG_ERROR("Not implemented!");
				continue;
			}
		}
	}

	void VehicleSystem::on_vehicle_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		vehicles_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}

	void VehicleSystem::on_vehicle_released(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		Entity entity = world->get_entity(entity_handle);

		if (VehicleBaseComponent* vehicle_component = VehicleUtils::get_vehicle_component(entity))
		{
			vehicle_component->vehicle->destroy();

			RELEASE_PTR(vehicle_component->vehicle)
			RELEASE_PTR(vehicle_component->vehicle_simulation_context)
		}
	}
}