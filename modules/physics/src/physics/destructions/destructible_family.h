#pragma once

#include "physics_api.h"

#include "physics/destructions/destruction_types.h"

#include <core/math.h>

#include <physics/destructions/blast_physx/NvBlastExtPxListener.h>
#include <physics/destructions/blast_physx/NvBlastExtPxStressSolver.h>

#include <extensions/shaders/NvBlastExtDamageShaders.h>
#include <stress/NvBlastExtStressSolver.h>

namespace Nv
{
	namespace Blast
	{
		class TkFamily;
		class ExtPxManager;
		class ExtPxActor;
	}
}

namespace physx
{
	class PxGeometry;
	class PxTransform;
}

namespace era_engine::physics
{
	class ERA_PHYSICS_API DestructibleFamily
	{
	public:
		struct Settings
		{
			bool stress_solver_enabled = false;
			bool stress_damage_enabled = false;
			bool damage_accelerator_enabled = false;
			NvBlastExtMaterial material;
			Nv::Blast::ExtStressSolverSettings stress_solver_settings;
		};

		DestructibleFamily(Nv::Blast::ExtPxManager& _px_manager, const ref<DestructibleAsset>& _blast_asset);
		virtual ~DestructibleFamily();

		void set_settings(const Settings& _settings);
		const Settings& get_settings() const;

		size_t get_family_size() const;
		uint32 get_actor_count() const;
		ref<DestructibleAsset> get_destructible_asset();

		const Nv::Blast::ExtPxFamily* get_family() const;

		const NvBlastExtMaterial& get_material() const;

		void reset_stress();

		void refresh_stress_solver_settings();
		void refresh_damage_accelerator_settings();

		void reload_stress_solver();

		bool overlap(const physx::PxGeometry& geometry, const physx::PxTransform& pose, std::function<void(Nv::Blast::ExtPxActor*, DestructibleFamily&)> hit_call);

		void update_pre_split(float dt);
		void update_after_split(float dt);

	protected:
		void initialize(const DestructibleAsset::ActorDesc& desc);

		virtual void on_actor_created(const Nv::Blast::ExtPxActor& actor) = 0;
		virtual void on_actor_update(const Nv::Blast::ExtPxActor& actor) = 0;
		virtual void on_actor_destroyed(const Nv::Blast::ExtPxActor& actor) = 0;
		virtual void on_actor_health_update(const Nv::Blast::ExtPxActor& actor) {}

		virtual void on_update() {}

		void process_actor_created(Nv::Blast::ExtPxFamily&, Nv::Blast::ExtPxActor& actor);
		void process_actor_destroyed(Nv::Blast::ExtPxFamily&, Nv::Blast::ExtPxActor& actor);

		class PxManagerListener : public Nv::Blast::ExtPxListener
		{
		public:
			PxManagerListener(DestructibleFamily* _family) : family(_family) {}

			virtual void onActorCreated(Nv::Blast::ExtPxFamily& _family, Nv::Blast::ExtPxActor& actor)
			{
				family->process_actor_created(_family, actor);

			}

			virtual void onActorDestroyed(Nv::Blast::ExtPxFamily& _family, Nv::Blast::ExtPxActor& actor)
			{
				family->process_actor_destroyed(_family, actor);
			}
		private:
			DestructibleFamily* family = nullptr;
		};

		friend class PxManagerListener;

		ref<DestructibleAsset> blast_asset;

		Nv::Blast::ExtPxManager& px_manager;

		Nv::Blast::TkFamily* tk_family = nullptr;
		Nv::Blast::ExtPxFamily* px_family = nullptr;
		Nv::Blast::ExtPxStressSolver* stress_solver = nullptr;

		PxManagerListener listener;
		trs initial_transform;

		std::set<Nv::Blast::ExtPxActor*> actors;
		std::set<const Nv::Blast::ExtPxActor*> actors_to_update_health;

		Settings settings;

		float stress_solve_time = 0.0f;

		uint32 total_visible_chunk_count = 0;
		size_t family_size = 0;

		bool is_spawned = false;
	};
}