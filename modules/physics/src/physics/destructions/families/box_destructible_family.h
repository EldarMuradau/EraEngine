#pragma once

#include "physics_api.h"

#include "physics/physx_api.h"
#include "physics/destructions/destructible_family.h"

#include <geometry/mesh.h>
#include <rendering/pbr_material.h>

namespace era_engine::physics
{
	class ERA_PHYSICS_API BoxDestructibleFamily : public DestructibleFamily
	{
	public:
		BoxDestructibleFamily(World* _world,
			Nv::Blast::ExtPxManager& _px_manager,
			DestructibleAsset* _blast_asset,
			const DestructibleAsset::ActorDesc& desc);
		~BoxDestructibleFamily() override;

	protected:
		void on_actor_created(const Nv::Blast::ExtPxActor& actor) override;
		void on_actor_update(const Nv::Blast::ExtPxActor& actor) override;
		void on_actor_destroyed(const Nv::Blast::ExtPxActor& actor) override;

		ref<MultiMesh> box_mesh;
		ref<pbr_material> box_render_material;
	};
}