#include "physics/pbd/private/pbd_cloth_system.h"
#include "physics/core/physics.h"
#include "physics/core/physics_utils.h"
#include "physics/body_component.h"
#include "physics/pbd/pbd_cloth_component.h"

#include <rendering/ecs/renderer_holder_root_component.h>
#include <rendering/debug_visualization.h>

#include <core/cpu_profiling.h>
#include <core/debug/debug_var.h>

#include <ecs/base_components/transform_component.h>
#include <ecs/rendering/cloth_render_component.h>
#include <ecs/update_groups.h>

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine::physics
{
	static DebugVar<bool> visualize_cloth_physics = DebugVar<bool>("physics.visualize_cloth_physics", false);

	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<PBDClothSystem>("PBDClothSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("physics")))
			.method("update", &PBDClothSystem::update)(metadata("update_group", update_types::BEFORE_PHYSICS))
			.method("debug_draw", &PBDClothSystem::debug_draw)(metadata("update_group", update_types::RENDER), metadata("After", std::vector<std::string>{"PhysicsVisualizationSystem::update"}));
	}

	PBDClothSystem::PBDClothSystem(World* _world)
		: System(_world)
	{
	}

	void PBDClothSystem::init()
	{
		entt::registry& registry = world->get_registry();
		registry.on_construct<PBDClothComponent>().connect<&PBDClothSystem::on_cloth_created>(this);
		registry.on_destroy<PBDClothComponent>().connect<&PBDClothSystem::on_cloth_released>(this);

		renderer_holder_rc = world->add_root_component<RendererHolderRootComponent>();
		ASSERT(renderer_holder_rc != nullptr);
	}

	void PBDClothSystem::update(float dt)
	{
		using namespace physx;

		ZoneScopedN("PBDClothSystem::update");

		process_added_clothes(dt);

		PxCudaContextManager* cuda_context_manager = PhysicsEngine::get_physics_core()->get_cuda_context_manager();

		for (auto&& [entity_handle, transform_component, cloth_component] : world->group(components_group<TransformComponent, PBDClothComponent>).each())
		{
			{
				PxScopedCudaLock lock{ *cuda_context_manager };

				PxVec4* positions = cloth_component.cloth_buffer->getPositionInvMasses();

				const PxU32 numParticles = cloth_component.cloth_buffer->getNbActiveParticles();

				cuda_context_manager->acquireContext();

				PxCudaContext* cudaContext = cuda_context_manager->getCudaContext();
				cudaContext->memcpyDtoH(cloth_component.native_pos_buffer, CUdeviceptr(positions), sizeof(PxVec4) * numParticles);

				cuda_context_manager->releaseContext();
			}
			
			cloth_component.sync_positions_buffers_locked();
		}
	}

	void PBDClothSystem::debug_draw(float dt)
	{
		using namespace physx;

		if (!visualize_cloth_physics)
		{
			return;
		}

		for (auto&& [entity_handle, transform_component, cloth_component] : world->group(components_group<TransformComponent, PBDClothComponent>).each())
		{
			const uint32 num_particles = cloth_component.num_x * cloth_component.num_z;

			for (size_t i = 0; i < num_particles; i++)
			{
				PxVec4 p_i = (PxVec4)cloth_component.native_pos_buffer[i];
				vec3 pos_i = vec3(p_i.x, p_i.y, p_i.z);
				renderPoint(pos_i, vec4(0.107f, 1.0f, 0.0f, 1.0f), renderer_holder_rc->ldrRenderPass, false);
			}
		}
	}

	void PBDClothSystem::process_added_clothes(float dt)
	{
		using namespace physx;

		ZoneScopedN("PBDClothSystem::process_added_clothes");

		ScopedSpinLock _lock{ sync };

		PxCudaContextManager* cuda_context_manager = PhysicsEngine::get_physics_core()->get_cuda_context_manager();

		for (auto iter = clothes_to_init.begin(); iter != clothes_to_init.end();)
		{
			Entity entity = world->get_entity(*iter);

			PBDClothComponent* cloth_component = entity.get_component<PBDClothComponent>();

			TransformComponent* transform_component = entity.get_component<TransformComponent>();

			ClothRenderComponent* render_component = entity.add_component<ClothRenderComponent>();
			render_component->get_data_internal = std::bind(&PBDClothComponent::get_render_data, cloth_component, std::placeholders::_1);

			// Test cloth data.
			{
				const uint32 num_particles = cloth_component->num_x * cloth_component->num_z;
				const PxU32 num_springs = (cloth_component->num_x - 1) * (cloth_component->num_z - 1) * 4 + (cloth_component->num_x - 1) + (cloth_component->num_z - 1);
				const PxU32 num_triangles = (cloth_component->num_x - 1) * (cloth_component->num_z - 1) * 2;

				const PxReal rest_offset = cloth_component->spacing;

				const PxReal stretch_stiffness = 10000.f;
				const PxReal shear_stiffness = 100.f;
				const PxReal spring_damping = 0.001f;

				cloth_component->material = PhysicsEngine::get_physics_core()->get_physics()->
					createPBDMaterial(0.8f, 0.05f, 1e+6f, 0.001f, 0.5f, 0.005f, 0.05f, 0.f, 0.f);

				cloth_component->particle_system = PhysicsEngine::get_physics_core()->get_physics()->
					createPBDParticleSystem(*cuda_context_manager);

				const PxReal particle_mass = cloth_component->total_mass / num_particles;
				cloth_component->particle_system->setRestOffset(rest_offset);
				cloth_component->particle_system->setContactOffset(rest_offset + 0.02f);
				cloth_component->particle_system->setParticleContactOffset(rest_offset + 0.02f);
				cloth_component->particle_system->setSolidRestOffset(rest_offset);
				cloth_component->particle_system->setFluidRestOffset(0.0f);
				cloth_component->particle_system->enableCCD(true);

				PxFilterData filter_data;
				filter_data.word0 = -1; // word0 = own ID
				filter_data.word1 = -1;  // word1 = ID mask to filter pairs that trigger a contact callback
				cloth_component->particle_system->setSimulationFilterData(filter_data);

				PhysicsEngine::execute_write([&]() {
					PhysicsEngine::get_physics_core()->get_scene()->addActor(*cloth_component->particle_system);
					});

				const PxU32 particle_phase = cloth_component->particle_system->createPhase(cloth_component->material, PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseSelfCollideFilter | PxParticlePhaseFlag::eParticlePhaseSelfCollide));

				ExtGpu::PxParticleClothBufferHelper* cloth_buffers = ExtGpu::PxCreateParticleClothBufferHelper(1, num_triangles, num_springs, num_particles, cuda_context_manager);

				PxU32* phase = cuda_context_manager->allocPinnedHostBuffer<PxU32>(num_particles);
				PxVec4* position_inv_mass = cuda_context_manager->allocPinnedHostBuffer<PxVec4>(num_particles);
				PxVec4* velocity = cuda_context_manager->allocPinnedHostBuffer<PxVec4>(num_particles);

				const vec3& position = transform_component->get_world_transform().position;

				PxReal x = position.x;
				PxReal y = position.y;
				PxReal z = position.z;

				PxArray<PxParticleSpring> springs;
				springs.reserve(num_springs);
				PxArray<PxU32> triangles;
				triangles.reserve(num_triangles * 3);

				for (PxU32 i = 0; i < cloth_component->num_x; ++i)
				{
					for (PxU32 j = 0; j < cloth_component->num_z; ++j)
					{
						const PxU32 index = i * cloth_component->num_z + j;

						PxVec4 pos(x, y, z, 1.0f / particle_mass);
						phase[index] = particle_phase;
						position_inv_mass[index] = pos;
						velocity[index] = PxVec4(0.0f);

						if (i > 0)
						{
							PxParticleSpring spring = { id(i - 1, j, cloth_component->num_z), id(i, j, cloth_component->num_z), rest_offset, stretch_stiffness, spring_damping, 0 };
							springs.pushBack(spring);
						}
						if (j > 0)
						{
							PxParticleSpring spring = { id(i, j - 1, cloth_component->num_z), id(i, j, cloth_component->num_z), rest_offset, stretch_stiffness, spring_damping, 0 };
							springs.pushBack(spring);
						}

						if (i > 0 && j > 0)
						{
							PxParticleSpring spring0 = { id(i - 1, j - 1, cloth_component->num_z), id(i, j, cloth_component->num_z), PxSqrt(2.0f) * rest_offset, shear_stiffness, spring_damping, 0 };
							springs.pushBack(spring0);
							PxParticleSpring spring1 = { id(i - 1, j, cloth_component->num_z), id(i, j - 1, cloth_component->num_z), PxSqrt(2.0f) * rest_offset, shear_stiffness, spring_damping, 0 };
							springs.pushBack(spring1);

							//Triangles are used to compute approximated aerodynamic forces for cloth falling down
							triangles.pushBack(id(i - 1, j - 1, cloth_component->num_z));
							triangles.pushBack(id(i - 1, j, cloth_component->num_z));
							triangles.pushBack(id(i, j - 1, cloth_component->num_z));

							triangles.pushBack(id(i - 1, j, cloth_component->num_z));
							triangles.pushBack(id(i, j - 1, cloth_component->num_z));
							triangles.pushBack(id(i, j, cloth_component->num_z));
						}

						z += rest_offset;
					}
					z = position.z;
					x += rest_offset;
				}

				PX_ASSERT(num_springs == springs.size());
				PX_ASSERT(num_triangles == triangles.size() / 3);

				cloth_buffers->addCloth(0.0f, 0.0f, 0.0f, triangles.begin(), num_triangles, springs.begin(), num_springs, position_inv_mass, num_particles);

				ExtGpu::PxParticleBufferDesc buffer_desc;
				buffer_desc.maxParticles = num_particles;
				buffer_desc.numActiveParticles = num_particles;
				buffer_desc.positions = position_inv_mass;
				buffer_desc.velocities = velocity;
				buffer_desc.phases = phase;

				const PxParticleClothDesc& cloth_desc = cloth_buffers->getParticleClothDesc();
				PxParticleClothPreProcessor* cloth_pre_processor = PxCreateParticleClothPreProcessor(cuda_context_manager);

				PxPartitionedParticleCloth output;
				cloth_pre_processor->partitionSprings(cloth_desc, output);
				cloth_pre_processor->release();

				cloth_component->cloth_buffer = ExtGpu::PxCreateAndPopulateParticleClothBuffer(buffer_desc, cloth_desc, output, cuda_context_manager);

				PhysicsEngine::execute_write([&]() {
					cloth_component->particle_system->addParticleBuffer(cloth_component->cloth_buffer);
					});

				cloth_buffers->release();

				cuda_context_manager->freePinnedHostBuffer(position_inv_mass);
				cuda_context_manager->freePinnedHostBuffer(velocity);
				cuda_context_manager->freePinnedHostBuffer(phase);

				uint32 size = num_particles * sizeof(PxVec4);
				cloth_component->allocator->initialize(0, (size + sizeof(PxVec4)) * 8.0f);
				cloth_component->native_pos_buffer = cloth_component->allocator->allocate<PxVec4>(num_particles, true);
				cloth_component->view_pos_buffer = cloth_component->allocator->allocate<vec3>(num_particles, true);
			}

			iter = clothes_to_init.erase(iter);
		}
	}

	void PBDClothSystem::on_cloth_created(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		clothes_to_init.push_back(static_cast<Entity::Handle>(entity_handle));
	}

	void PBDClothSystem::on_cloth_released(entt::registry& registry, entt::entity entity_handle)
	{
		ScopedSpinLock _lock{ sync };

		Entity entity = world->get_entity(entity_handle);

		PBDClothComponent* pbd_cloth_component = entity.get_component<PBDClothComponent>();

		if (pbd_cloth_component != nullptr)
		{
			PhysicsEngine::execute_write([&]() {
				PhysicsEngine::get_physics_core()->get_scene()->removeActor(*pbd_cloth_component->particle_system);
				pbd_cloth_component->particle_system->removeParticleBuffer(pbd_cloth_component->cloth_buffer);

				PX_RELEASE(pbd_cloth_component->particle_system)

				PX_RELEASE(pbd_cloth_component->cloth_buffer)
				PX_RELEASE(pbd_cloth_component->material)
				});

			pbd_cloth_component->allocator->reset(true);

		}
	}
}