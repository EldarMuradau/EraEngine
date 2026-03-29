#include "physics/destructions/destructible_family.h"
#include "physics/physx_api.h"
#include "physics/core/physics.h"
#include "physics/destructions/blast_physx/NvBlastExtPxManager.h"
#include "physics/destructions/blast_physx/NvBlastExtPxAsset.h"
#include "physics/destructions/blast_physx/NvBlastExtPxActor.h"
#include "physics/destructions/blast_physx/NvBlastExtPxFamily.h"

#include <core/cpu_profiling.h>

#include "NvBlast.h"
#include "NvBlastTkFamily.h"
#include "NvBlastTkActor.h"
#include "NvBlastTkAsset.h"
#include "NvBlastTkJoint.h"

namespace era_engine::physics
{
	static constexpr float RIGIDBODY_DENSITY = 2000.0f;

	class BlastOverlapCallback : public physx::PxOverlapCallback
	{
	public:
		BlastOverlapCallback(Nv::Blast::ExtPxManager& _px_manager, std::set<Nv::Blast::ExtPxActor*>& _actor_buffer)
			: px_manager(_px_manager), actor_buffer(_actor_buffer), physx::PxOverlapCallback(hit_buffer, sizeof(hit_buffer) / sizeof(hit_buffer[0])) {}

		physx::PxAgain processTouches(const physx::PxOverlapHit* buffer, physx::PxU32 nb_hits)
		{
			using namespace physx;
			using namespace Nv::Blast;

			for (PxU32 i = 0; i < nb_hits; ++i)
			{
				PxRigidDynamic* rigid_dynamic = buffer[i].actor->is<PxRigidDynamic>();
				if (rigid_dynamic != nullptr)
				{
					ExtPxActor* actor = px_manager.getActorFromPhysXActor(*rigid_dynamic);
					if (actor != nullptr)
					{
						actor_buffer.insert(actor);
					}
				}
			}
			return true;
		}

	private:
		physx::PxOverlapHit hit_buffer[1024];
		Nv::Blast::ExtPxManager& px_manager;
		std::set<Nv::Blast::ExtPxActor*>& actor_buffer;
	};

	DestructibleFamily::DestructibleFamily(Nv::Blast::ExtPxManager& _px_manager, const ref<DestructibleAsset>& _blast_asset)
		: px_manager(_px_manager)
		, blast_asset(_blast_asset)
		, listener(this)
	{
		settings.stress_solver_enabled = false;
		settings.stress_damage_enabled = false;
		settings.damage_accelerator_enabled = true;
	}

	DestructibleFamily::~DestructibleFamily()
	{
		if (stress_solver != nullptr)
		{
			stress_solver->release();
		}
		if (px_family != nullptr)
		{
			px_family->unsubscribe(listener);
			px_family->release();
		}
		//Self released
		//if (tk_family != nullptr)
		//{
		//	tk_family->release();
		//}
	}

	void DestructibleFamily::set_settings(const Settings& _settings)
	{
		const bool reload_stress_solver_needed = (settings.stress_solver_enabled != _settings.stress_solver_enabled);

		settings = _settings;
		refresh_stress_solver_settings();
		refresh_damage_accelerator_settings();

		if (reload_stress_solver_needed)
		{
			reload_stress_solver();
		}

		px_family->setMaterial(&settings.material);
	}

	const DestructibleFamily::Settings& DestructibleFamily::get_settings() const
	{
		return settings;
	}

	size_t DestructibleFamily::get_family_size() const
	{
		return family_size;
	}

	uint32 DestructibleFamily::get_actor_count() const
	{
		return static_cast<uint32>(tk_family->getActorCount());
	}

	ref<DestructibleAsset> DestructibleFamily::get_destructible_asset()
	{
		return blast_asset;
	}

	const Nv::Blast::ExtPxFamily* DestructibleFamily::get_family() const
	{
		return px_family;
	}

	const NvBlastExtMaterial& DestructibleFamily::get_material() const
	{
		return settings.material;
	}
	
	void DestructibleFamily::reset_stress()
	{
		if (stress_solver != nullptr)
		{
			stress_solver->getSolver().reset();
		}
	}

	void DestructibleFamily::refresh_stress_solver_settings()
	{
		if (stress_solver != nullptr)
		{
			stress_solver->getSolver().setSettings(settings.stress_solver_settings);
		}
	}

	void DestructibleFamily::refresh_damage_accelerator_settings()
	{
		px_family->getPxAsset().setAccelerator(settings.damage_accelerator_enabled ? blast_asset->damage_accelerator : nullptr);
	}

	void DestructibleFamily::reload_stress_solver()
	{
		using namespace Nv::Blast;

		if (stress_solver != nullptr)
		{
			stress_solver->release();
			stress_solver = nullptr;
		}

		if (settings.stress_solver_enabled)
		{
			stress_solver = ExtPxStressSolver::create(*px_family, settings.stress_solver_settings);
			px_family->userData = stress_solver;
		}
	}

	bool DestructibleFamily::overlap(const physx::PxGeometry& geometry, const physx::PxTransform& pose, std::function<void(Nv::Blast::ExtPxActor*, DestructibleFamily&)> hit_call)
	{
		using namespace Nv::Blast;

		std::set<ExtPxActor*> actors_to_damage;
		BlastOverlapCallback overlap_callback(px_manager, actors_to_damage);
		PhysicsEngine::get_physics_core()->get_scene()->overlap(geometry, pose, overlap_callback);

		for (ExtPxActor* actor : actors_to_damage)
		{
			hit_call(actor, *this);
		}

		return !actors_to_damage.empty();
	}

	void DestructibleFamily::update_pre_split(float dt)
	{
		using namespace Nv::Blast;
		using namespace physx;

		if (!is_spawned)
		{
			ExtPxSpawnSettings spawn_settings = {
				PhysicsEngine::get_physics_core()->get_scene(),
				PhysicsEngine::get_physics_core()->get_default_material()->get_native_material(),
				RIGIDBODY_DENSITY
			};

			px_family->spawn(create_PxTransform(initial_transform), PxVec3(1.0f), spawn_settings);
			reload_stress_solver();

			is_spawned = true;
		}

		// Collect potential actors to health update.
		actors_to_update_health.clear();

		for (const ExtPxActor* actor : actors)
		{
			if (actor->getTkActor().isPending())
			{
				actors_to_update_health.emplace(actor);
			}
		}
	}

	void DestructibleFamily::update_after_split(float dt)
	{
		using namespace Nv::Blast;

		{
			ZoneScopedN("Actor Health Update");
			for (const ExtPxActor* actor : actors)
			{
				on_actor_update(*actor);

				if (actors_to_update_health.find(actor) != actors_to_update_health.end())
				{
					on_actor_health_update(*actor);
				}
			}
		}
		
		{
			ZoneScopedN("Stress Solver");
			stress_solve_time = 0;
			if (stress_solver != nullptr)
			{
				auto before = std::chrono::steady_clock::now();
				stress_solver->update(settings.stress_damage_enabled);
				auto after = std::chrono::steady_clock::now();

				auto elapsed = after - before;

				float solver_elapsed_dt = std::chrono::duration<float>(elapsed).count();
				stress_solve_time += solver_elapsed_dt;
			}
		}

		{
			ZoneScopedN("Actors Update");
			on_update();
		}

		px_family->postSplitUpdate();
	}

	void DestructibleFamily::initialize(const DestructibleAsset::ActorDesc& desc)
	{
		using namespace Nv::Blast;

		ExtPxFamilyDesc family_desc{};
		family_desc.actorDesc = nullptr; // If you use it one day, consider changing code which needs get_bond_health_max() from DestructibleAsset.
		family_desc.group = desc.group;
		family_desc.pxAsset = blast_asset->px_asset;
		px_family = px_manager.createFamily(family_desc);
		px_family->setMaterial(&settings.material);

		tk_family = &px_family->getTkFamily();
		tk_family->setID(desc.id);

		refresh_damage_accelerator_settings();

		family_size = NvBlastFamilyGetSize(tk_family->getFamilyLL(), nullptr);

		px_family->subscribe(listener);

		initial_transform = physx::create_trs(desc.transform);
	}

	void DestructibleFamily::process_actor_created(Nv::Blast::ExtPxFamily&, Nv::Blast::ExtPxActor& actor)
	{
		total_visible_chunk_count += actor.getChunkCount();
		actors.emplace(&actor);

		on_actor_created(actor);
		on_actor_health_update(actor);
	}

	void DestructibleFamily::process_actor_destroyed(Nv::Blast::ExtPxFamily&, Nv::Blast::ExtPxActor& actor)
	{
		total_visible_chunk_count -= actor.getChunkCount();
		on_actor_destroyed(actor);

		actors.erase(actors.find(&actor));
	}

}