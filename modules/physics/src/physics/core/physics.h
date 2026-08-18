#pragma once

#include "physics/physx_api.h"
#include "physics/core/physics_types.h"
#include "physics/material.h"
#include "physics/destructions/blast_core.h"

#include <core/memory.h>
#include <core/math.h>

#include <ecs/entity.h>

#include <concurrentqueue/concurrentqueue.h>

namespace era_engine
{
	class EditorScene;
	class PhysicsModule;
}

namespace era_engine::physics
{
	class BodyComponent;
	class ShapeComponent;

	class ERA_PHYSICS_API PhysicsDescriptor
	{
	public:
		physx::PxBroadPhaseType::Enum broad_phase = physx::PxBroadPhaseType::ePABP;
		bool enable_pvd = true;
		bool enable_tgs_solver = true;
		const char* omni_pvd_path = "out.ovd";
	};

	class ERA_PHYSICS_API Physics final
	{
	public:
		Physics(const PhysicsDescriptor& _descriptor = {});
		~Physics();

		void init_scene();

		physx::PxScene* get_scene() const;
		physx::PxPhysics* get_physics() const;
		const ref<PhysicsMaterial>& get_default_material() const;

		const PhysicsDescriptor& get_descriptor() const;

		ref<PhysicsMaterial> create_material(float restitution = 0.6f,
			float static_friction = 0.8f,
			float dynamic_friction = 0.8f);

		physx::PxCudaContextManager* get_cuda_context_manager() const;
		physx::PxCpuDispatcher* get_cpu_dispatcher() const;
		physx::PxTolerancesScale get_tolerance_scale() const;

		ref<BlastCore> get_blast_core() const;

		bool is_gpu() const;

		void release_locked();

		void start();
		void update_locked(float dt);

		void clear_collisions();

		void start_simulation(float dt);
		void end_simulation(float dt);

		void reset_actors_velocity_and_inertia_locked();

		void add_shape_to_entity_data(ShapeComponent* shape);
		void remove_shape_from_entity_data(ShapeComponent* shape);

		void add_actor_locked(BodyComponent* actor, physx::PxRigidActor* physx_actor);
		void remove_actor_locked(BodyComponent* actor);

		void release_scene_locked();

		void explode(const vec3& world_pos, float damage_radius, float explosive_impulse);

	private:
		void sync_transforms_locked();
		void process_simulation_event_callbacks();

	public:
		std::atomic_uint32_t nb_active_actors{};

		std::set<BodyComponent*> actors;

		std::unordered_map<Entity::Handle, std::vector<ShapeComponent*>> colliders_map;
		std::unordered_map<physx::PxRigidActor*, BodyComponent*> actors_map;

		ref<SimulationEventCallback> simulation_event_callback = nullptr;

		SpinLock sync;

	private:
		PhysicsDescriptor descriptor;
		Allocator allocator;

		physx::PxScene* scene = nullptr;

		void* scratch_mem_block = nullptr;

		physx::PxPhysics* physics = nullptr;

		physx::PxPvd* pvd = nullptr;

		ref<PhysicsMaterial> default_material = nullptr;

		physx::PxFoundation* foundation = nullptr;

		physx::PxDefaultCpuDispatcher* dispatcher = nullptr;

		physx::PxCudaContextManager* cuda_context_manager = nullptr;

		physx::PxPvdTransport* transport = nullptr;

		physx::PxControllerManager* cct_manager = nullptr;

		PhysicsAllocatorCallback* allocator_callback = nullptr;
		ErrorReporter error_reporter;

		ProfilerCallback profiler_callback;

		SimulationFilterCallback simulation_filter_callback;

		std::vector<ref<PhysicsMaterial>> materials;

		physx::PxTolerancesScale tolerance_scale;

		const uint32 nb_cpu_dispatcher_threads = 4;
		static constexpr uint64 scratch_mem_block_size = MB(64U);

		ref<BlastCore> blast_core;

		friend class PhysicsSystem;
	};

	class ERA_PHYSICS_API PhysicsEngine final
	{
		PhysicsEngine() = delete;

	public:

		static ref<Physics> get_physics_core();

		static void execute_read(const std::function<void()>& func);
		static void execute_write(const std::function<void()>& func);

	private:
		static inline ref<Physics> physics_core = nullptr;

		friend PhysicsModule;
	};
}