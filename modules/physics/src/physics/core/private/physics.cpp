#include "physics/core/physics.h"
#include "physics/shape_component.h"
#include "physics/body_component.h"
#include "physics/soft_body.h"
#include "physics/aggregate_holder_component.h"
#include "physics/aggregate.h"
#include "physics/cct_component.h"

#include "core/cpu_profiling.h"
#include "core/event_queue.h"

#include "ecs/base_components/transform_component.h"
#include "ecs/world.h"
#include "ecs/editor/editor_scene.h"

#include "omnipvd/PxOmniPvd.h"

#include "pvdruntime/OmniPvdWriter.h"
#include "pvdruntime/OmniPvdFileWriteStream.h"

namespace era_engine::physics
{
	static physx::PxOmniPvd* omni_pvd = NULL;

	static physx::PxFilterFlags contact_report_filter_shader(
		physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filter_data0,
		physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filter_data1,
		physx::PxPairFlags& pair_flags, const void* constant_block, physx::PxU32 constant_block_size)
	{
		UNUSED(constant_block_size);
		UNUSED(constant_block);

		if (physx::PxFilterObjectIsTrigger(attributes0) || physx::PxFilterObjectIsTrigger(attributes1))
		{
			pair_flags = physx::PxPairFlag::eTRIGGER_DEFAULT;
			return physx::PxFilterFlag::eDEFAULT;
		}

		if ((filter_data0.word0 & filter_data1.word1) && (filter_data1.word0 & filter_data0.word1))
		{
			pair_flags = physx::PxPairFlag::eCONTACT_DEFAULT;
			pair_flags |= physx::PxPairFlag::eDETECT_CCD_CONTACT;
			pair_flags |= physx::PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pair_flags |= physx::PxPairFlag::eNOTIFY_TOUCH_LOST;
			pair_flags |= physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS;
			pair_flags |= physx::PxPairFlag::ePOST_SOLVER_VELOCITY;
			pair_flags |= physx::PxPairFlag::eNOTIFY_CONTACT_POINTS;
			pair_flags |= physx::PxPairFlag::eSOLVE_CONTACT;
			pair_flags |= physx::PxPairFlag::eDETECT_DISCRETE_CONTACT;

			return physx::PxFilterFlag::eDEFAULT;
		}

		return physx::PxFilterFlag::eSUPPRESS;
	}

	static void clear_collider_from_collection(const physics::BodyComponent* collider,
		physx::PxArray<SimulationEventCallback::CollidersPair>& collection)
	{
		const auto c = &collection[0];
		for (uint32 i = 0; i < collection.size(); i++)
		{
			const SimulationEventCallback::CollidersPair cc = c[i];
			if (cc.first == collider || cc.second == collider)
			{
				collection.remove(i--);
				if (collection.empty())
				{
					break;
				}
			}
		}
	}

	Physics::Physics(const PhysicsDescriptor& _descriptor)
		: descriptor(_descriptor)
	{
		using namespace physx;

		allocator.initialize(MB(256U));

		allocator_callback = new PhysicsAllocatorCallback();
		foundation = PxCreateFoundation(PX_PHYSICS_VERSION, *allocator_callback, error_reporter);

		if (foundation == nullptr)
		{
			throw std::exception("Failed to create {PxFoundation}. Error in {PhysicsEngine} ctor.");
		}

		if (descriptor.broad_phase == PxBroadPhaseType::eGPU && PxGetSuggestedCudaDeviceOrdinal(foundation->getErrorCallback()) < 0)
		{
			throw std::exception("Failed to create {PxFoundation}. Error in {PhysicsEngine} ctor.");
		}

		tolerance_scale.length = 1.0f;
		tolerance_scale.speed = 10.0f;

		physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, tolerance_scale, true, pvd, omni_pvd);

		if (!PxInitExtensions(*physics, pvd))
		{
			LOG_ERROR("Physics> Failed to initialize extensions.");
		}

		default_material = create_material(0.8f, 0.8f, 0.6f);

		if (descriptor.enable_pvd && physics->getOmniPvd())
		{
			const bool result = omni_pvd->startSampling();
			ASSERT(result);
		}

		dispatcher = PxDefaultCpuDispatcherCreate(nb_cpu_dispatcher_threads);

#if PX_VEHICLE
		if (!PxInitVehicleSDK(*physics))
		{
			LOG_ERROR("Physics> Failed to initialize PxVehicleSDK.");
		}

		PxVehicleSetBasisVectors(PxVec3(0, 1, 0), PxVec3(0, 0, 1));
		PxVehicleSetUpdateMode(PxVehicleUpdateMode::eVELOCITY_CHANGE);
#endif
	}

	Physics::~Physics()
	{
		release();

		allocator.reset();
	}

	void Physics::init_scene()
	{
		using namespace physx;

		PxSceneDesc scene_desc(tolerance_scale);
		scene_desc.gravity = gravity;
		scene_desc.cpuDispatcher = dispatcher;
		if (descriptor.enable_tgs_solver)
		{
			scene_desc.solverType = PxSolverType::eTGS;
		}
		scene_desc.flags |= PxSceneFlag::eENABLE_CCD;
		scene_desc.flags |= PxSceneFlag::eENABLE_ACTIVE_ACTORS;
		scene_desc.flags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
		scene_desc.flags |= PxSceneFlag::eENABLE_STABILIZATION;
		scene_desc.flags |= PxSceneFlag::eENABLE_PCM;

		if (descriptor.broad_phase == PxBroadPhaseType::eGPU)
		{
			PxCudaContextManagerDesc cuda_context_manager_desc;
			cuda_context_manager = PxCreateCudaContextManager(*foundation, cuda_context_manager_desc, &profiler_callback);
			scene_desc.cudaContextManager = cuda_context_manager;
			scene_desc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
			scene_desc.gpuMaxNumPartitions = 8;
		}

		scene_desc.broadPhaseType = descriptor.broad_phase;

		scene_desc.filterShader = contact_report_filter_shader;

		simulation_event_callback = make_ref<SimulationEventCallback>();

		scene_desc.simulationEventCallback = simulation_event_callback.get();

		scene = physics->createScene(scene_desc);

#if defined(VISUALIZE_PHYSICS)

		scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
		scene->setVisualizationParameter(PxVisualizationParameter::eCONTACT_POINT, 1.0f);
		scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_FNORMALS, 1.0f);
		scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);

		if (descriptor.enable_pvd)
		{
			pvd = PxCreatePvd(*foundation);

			PxPvdSceneClient* pvd_client = scene->getScenePvdClient();
			if (pvd_client)
			{
				pvd_client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
				pvd_client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
				pvd_client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
			}

			transport = PxDefaultPvdSocketTransportCreate("localhost", 5425, 10);
			if (transport)
			{
				pvd->connect(*transport,
					PxPvdInstrumentationFlag::ePROFILE
				);
			}

			omni_pvd = PxCreateOmniPvd(*foundation);
			if (!omni_pvd)
			{
				LOG_ERROR("Error: could not create PxOmniPvd!");
				return;
			}
			OmniPvdWriter* omniWriter = omni_pvd->getWriter();
			if (!omniWriter)
			{
				LOG_ERROR("Error: could not get an instance of PxOmniPvdWriter!");
				return;
			}
			OmniPvdFileWriteStream* fStream = omni_pvd->getFileWriteStream();
			if (!fStream)
			{
				LOG_ERROR("Error: could not get an instance of PxOmniPvdFileWriteStream!");
				return;
			}
			fStream->setFileName(descriptor.omni_pvd_path);
			omniWriter->setWriteStream(static_cast<OmniPvdWriteStream&>(*fStream));
		}
#endif

		static constexpr uint64 align = 16U;
		scratch_mem_block = allocator.allocate(scratch_mem_block_size, align, true);

		cct_manager = PxCreateControllerManager(*scene);
	}

	physx::PxScene* Physics::get_scene() const
	{
		return scene;
	}

	physx::PxPhysics* Physics::get_physics() const
	{
		return physics;
	}

	ref<PhysicsMaterial> Physics::get_default_material() const
	{
		return default_material;
	}

	ref<PhysicsMaterial> Physics::create_material(float restitution, float static_friction, float dynamic_friction)
	{
		ref<PhysicsMaterial> material = make_ref<PhysicsMaterial>(physics, restitution, static_friction, dynamic_friction);
		materials.emplace_back(material);
		return material;
	}

	physx::PxCudaContextManager* Physics::get_cuda_context_manager() const
	{
		return cuda_context_manager;
	}

	physx::PxCpuDispatcher* Physics::get_cpu_dispatcher() const
	{
		return dispatcher;
	}

	physx::PxTolerancesScale Physics::get_tolerance_scale() const
	{
		return tolerance_scale;
	}

	bool Physics::is_gpu() const
	{
		return descriptor.broad_phase == physx::PxBroadPhaseType::eGPU;
	}

	void Physics::release()
	{
		ScopedSpinLock lock{ sync };

		release_scene();

#if PX_VEHICLE
		physx::PxCloseVehicleSDK();
#endif

		PxCloseExtensions();

		{
			PX_RELEASE(default_material)
			PX_RELEASE(cct_manager)
			PX_RELEASE(physics)
			PX_RELEASE(dispatcher)

#if defined(_DEBUG)

			PX_RELEASE(pvd)
			PX_RELEASE(omni_pvd)
			PX_RELEASE(transport)

#endif

			PX_RELEASE(cuda_context_manager)
			PX_RELEASE(foundation)
		}

		delete allocator_callback;
	}

	void Physics::start()
	{
	}

	void Physics::update(float dt)
	{
		start_simulation(dt);
		end_simulation(dt);
	}

	void Physics::clear_collisions()
	{
		simulation_event_callback->clear();
	}

	void Physics::start_simulation(float dt)
	{
		using namespace physx;

		{
			ZoneScopedN("Physics::simulate");

			scene->simulate(dt, nullptr, scratch_mem_block, scratch_mem_block_size);
		}

		{
			ZoneScopedN("Physics::fetchResults");
			scene->fetchResults(true);
		}
	}

	void Physics::end_simulation(float dt)
	{
		{
			ZoneScopedN("Physics::process_simulation_event_callbacks");
			process_simulation_event_callbacks();
		}

		{
			ZoneScopedN("Physics::sync_transforms");
			sync_transforms();
		}
	}

	void Physics::reset_actors_velocity_and_inertia()
	{
		using namespace physx;

		PxU32 nb_active_actors = 0;
		PxActor** active_actors = scene->getActiveActors(nb_active_actors);

		for (size_t i = 0; i < nb_active_actors; ++i)
		{
			if (auto rd = active_actors[i]->is<PxRigidDynamic>())
			{
				rd->setAngularVelocity(PxVec3(0.0f));
				rd->setLinearVelocity(PxVec3(0.0f));
			}
		}
	}

	void Physics::add_shape_to_entity_data(ShapeComponent* shape)
	{
		ScopedSpinLock lock{ sync };
		colliders_map[shape->get_handle()].push_back(shape);
	}

	void Physics::remove_shape_from_entity_data(ShapeComponent* shape)
	{
		ScopedSpinLock lock{ sync };

		auto& shapes = colliders_map[shape->get_handle()];
		const auto& end = shapes.end();
		for (auto iter = shapes.begin(); iter != end; ++iter)
		{
			if (*iter == shape)
			{
				shapes.erase(iter);
				break;
			}
		}
	}

	void Physics::add_actor(BodyComponent* actor, physx::PxRigidActor* physx_actor)
	{
		using namespace physx;

		ScopedSpinLock l{ sync };

		if (AggregateHolderComponent* aggregate_component = actor->get_world()->get_entity(actor->get_entity().get_parent_handle()).get_component_if_exists<AggregateHolderComponent>())
		{
			aggregate_component->aggregate->add_actor(physx_actor);
		}
		else
		{
			scene->addActor(*physx_actor);
		}

		actors.emplace(actor);
		actors_map.insert(std::make_pair(physx_actor, actor));
	}

	void Physics::remove_actor(BodyComponent* actor)
	{
		using namespace physx;

		ScopedSpinLock l{ sync };
		actors.erase(actor);
		actors_map.erase(actor->get_rigid_actor());

		if (AggregateHolderComponent* aggregate_component = actor->get_world()->get_entity(actor->get_entity().get_parent_handle()).get_component<AggregateHolderComponent>())
		{
			aggregate_component->aggregate->remove_actor(actor->get_rigid_actor());
		}
		else
		{
			scene->removeActor(*actor->get_rigid_actor());
		}
	}

	void Physics::release_scene()
	{
		using namespace physx;
		ScopedSpinLock l{ sync };

		if (scene)
		{
			scene->flushSimulation();
		}

		for (auto& actor : actors)
		{
			scene->removeActor(*actor->get_rigid_actor());
		}

		actors.clear();
		actors_map.clear();
		colliders_map.clear();

		nb_active_actors = 0;

		PX_RELEASE(scene)
	}

	void Physics::explode(const vec3& world_pos, float damage_radius, float explosive_impulse)
	{
		using namespace physx;
		PxVec3 pos = create_PxVec3(world_pos);
		ExplodeOverlapCallback overlap_callback(pos, damage_radius, explosive_impulse);
		scene->overlap(PxSphereGeometry(damage_radius), PxTransform(pos), overlap_callback);
	}

	void Physics::sync_transforms()
	{
		using namespace physx;

		uint32 temp_nb = 0;
		PxActor** active_actors = scene->getActiveActors(temp_nb);
		nb_active_actors.store(temp_nb, std::memory_order_relaxed);

		for (size_t i = 0; i < nb_active_actors; ++i)
		{
			if (!active_actors[i]->userData)
			{
				continue;
			}

			if (auto rb = active_actors[i]->is<PxRigidDynamic>())
			{
				Component* data = static_cast<Component*>(active_actors[i]->userData);

				if (data == nullptr)
				{
					continue;
				}

				if (data->get_type() == rttr::type::get<CharacterControllerComponent>())
				{
					continue;
				}

				// Check joint shape sync.
				{
					auto& colliders = colliders_map[data->get_handle()];

					if (!colliders.empty())
					{
						auto& shape_component = colliders.front(); // Only for single-shape entities.

						if (shape_component->attachment_active)
						{
							continue;
						}
					}
				}

				TransformComponent* transform = data->get_entity().get_component<TransformComponent>();

				const auto& pxt = rb->getGlobalPose();
				const auto& pos = pxt.p;
				const auto& rot = pxt.q;

				trs new_transform(create_vec3(pos), create_quat(rot), transform->get_local_transform().scale);

				transform->set_world_transform(new_transform);
			}
		}

		if (is_gpu())
		{
			for (auto&& [name, world] : get_worlds())
			{
				for (auto [entity_handle, transform, soft_body] : world->group(components_group<TransformComponent, SoftBodyComponent>).each())
				{
					soft_body.copy_deformed_vertices_from_gpu_async(0);
				}
			}
		}
	}

	void Physics::process_simulation_event_callbacks()
	{
		simulation_event_callback->sendCollisionEvents();
		simulation_event_callback->sendTriggerEvents();
		simulation_event_callback->sendJointEvents();
	}

}