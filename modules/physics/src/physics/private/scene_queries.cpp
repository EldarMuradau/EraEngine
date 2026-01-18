#include "physics/scene_queries.h"
#include "physics/core/physics.h"
#include "physics/shape_component.h"
#include "physics/collisions_holder_root_component.h"

#include <ecs/world.h>

namespace era_engine::physics
{

	static void create_location_hit(const physx::PxLocationHit& physx_hit, SceneQueryHitType hit_type, SceneQueryPositionedHit& out_hit)
	{
		using namespace physx;

		out_hit.type = hit_type;
		out_hit.position = create_vec3(physx_hit.position);
		out_hit.normal = create_vec3(physx_hit.normal);
		out_hit.distance = physx_hit.distance;
	}

	static uint32 create_phyx_query_flags(const bool any, SceneQueryFilterFlag flags)
	{
		using namespace physx;

		uint32 physx_query_flags = 0;

		if (any)
		{
			physx_query_flags |= PxQueryFlag::Enum::eANY_HIT;
		}
		if ((flags & SceneQueryFilterFlag::DYNAMICS) == SceneQueryFilterFlag::DYNAMICS)
		{
			physx_query_flags |= PxQueryFlag::Enum::eDYNAMIC;
		}
		if ((flags & SceneQueryFilterFlag::STATICS) == SceneQueryFilterFlag::STATICS)
		{
			physx_query_flags |= PxQueryFlag::Enum::eSTATIC;
		}
		if ((flags & SceneQueryFilterFlag::PREFILTER) == SceneQueryFilterFlag::PREFILTER)
		{
			physx_query_flags |= PxQueryFlag::Enum::ePREFILTER;
		}
		if ((flags & SceneQueryFilterFlag::POSTFILTER) == SceneQueryFilterFlag::POSTFILTER)
		{
			physx_query_flags |= PxQueryFlag::Enum::ePOSTFILTER;
		}

		return physx_query_flags;
	}

	static physx::PxHitFlags create_physx_hit_flags(const bool use_single_sided_mesh_backface_hits, const uint32 buffer_size)
	{
		using namespace physx;

		PxHitFlags hit_flags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL;

		if (use_single_sided_mesh_backface_hits)
		{
			hit_flags |= physx::PxHitFlag::eMESH_BOTH_SIDES;
		}

		if (buffer_size > 1)
		{
			hit_flags |= physx::PxHitFlag::eMESH_MULTIPLE;
		}

		return hit_flags;
	}

	class PhysXQueryFilterCallback final : public physx::PxQueryFilterCallback
	{
	public:
		PhysXQueryFilterCallback(SceneQueryType query_type, SceneQueryFilterCallback* callback)
			: query_type(query_type)
			, callback(callback)
		{
		}

		physx::PxQueryHitType::Enum preFilter(const physx::PxFilterData& filter_data,
			const physx::PxShape* shape,
			const physx::PxRigidActor* actor,
			physx::PxHitFlags& query_flags) override
		{
			using namespace physx;

			ASSERT(callback != nullptr);

			ASSERT(shape != nullptr);
			if (shape->userData != nullptr)
			{
				const Component* component = static_cast<const Component*>(shape->userData);

				SceneQueryHitType hit_type = callback->prefilter(dynamic_cast<const ShapeComponent*>(component));
				return static_cast<PxQueryHitType::Enum>(hit_type);
			}

			return PxQueryHitType::eNONE;
		}

		physx::PxQueryHitType::Enum postFilter(const physx::PxFilterData& filter_data,
			const physx::PxQueryHit& physx_hit,
			const physx::PxShape* shape,
			const physx::PxRigidActor* actor) override
		{
			using namespace physx;

			ASSERT(callback != nullptr);

			SceneQueryHitType hit_type = SceneQueryHitType::NONE;

			Component* component = static_cast<Component*>(shape->userData);

			if (query_type == SceneQueryType::OVERLAP)
			{
				SceneQueryHit hit;
				hit.shape_component = dynamic_cast<ShapeComponent*>(component);
				hit.type = SceneQueryHitType::NONE;
				hit_type = callback->postfilter(hit);

				ASSERT(hit_type != SceneQueryHitType::BLOCK);
			}
			else
			{
				SceneQueryPositionedHit hit;
				hit.shape_component = dynamic_cast<ShapeComponent*>(component);
				create_location_hit(static_cast<const PxLocationHit&>(physx_hit), SceneQueryHitType::NONE, hit);
				hit_type = callback->postfilter(hit);
			}

			return static_cast<PxQueryHitType::Enum>(hit_type);
		}

	private:
		SceneQueryType query_type;
		SceneQueryFilterCallback* const callback = nullptr;
	};

	SceneQueryGeometry::SceneQueryGeometry()
		: type(Type::SPHERE)
		, sphere_radius(0.05f)
	{
	}

	static uint32 raycast_internal(
		const World* world,
		const ray& ray,
		float ray_distance,
		bool any,
		bool use_single_sided_mesh_backface_hits,
		SceneQueryFilterFlag flags,
		SceneQueryFilterCallback* callback,
		physx::PxRaycastBuffer* physx_buffer_out,
		uint32 result_buffer_size,
		SceneQueryPositionedHit* result_buffer_out)
	{
		using namespace physx;

		ASSERT(world != nullptr);
		ASSERT(ray_distance >= 0.0f);

		ASSERT(((flags & SceneQueryFilterFlag::STATICS) == SceneQueryFilterFlag::STATICS) ||
			((flags & SceneQueryFilterFlag::DYNAMICS) == SceneQueryFilterFlag::DYNAMICS));

		if (callback != nullptr)
		{
			ASSERT(((flags & SceneQueryFilterFlag::PREFILTER) == SceneQueryFilterFlag::PREFILTER) ||
				((flags & SceneQueryFilterFlag::POSTFILTER) == SceneQueryFilterFlag::POSTFILTER));
		}

		ASSERT(result_buffer_size > 0);
		ASSERT(result_buffer_out != nullptr);

		ref<Physics> physics = PhysicsHolder::physics_ref;

		const PxScene* physx_scene = physics->get_scene();
		if (physx_scene == nullptr)
		{
			return 0;
		}

		uint32 physx_query_flags = create_phyx_query_flags(any, flags);

		PxHitFlags hit_flags = create_physx_hit_flags(use_single_sided_mesh_backface_hits, result_buffer_size);

		PxQueryFilterData physx_filter_data(static_cast<PxQueryFlag::Enum>(physx_query_flags));

		const vec3 normalized_ray_direction = normalize(ray.direction);
		PhysXQueryFilterCallback physx_proxy_callback(SceneQueryType::RAYCAST, callback);
		const bool hit = physx_scene->raycast(
			create_PxVec3(ray.origin),
			create_PxVec3(normalized_ray_direction),
			ray_distance,
			*physx_buffer_out,
			hit_flags,
			physx_filter_data,
			(callback == nullptr) ? nullptr : &physx_proxy_callback);

		uint32 hit_index = 0;

		if (hit)
		{
			if (physx_buffer_out->hasBlock)
			{
				create_location_hit(physx_buffer_out->block, SceneQueryHitType::BLOCK, result_buffer_out[hit_index]);
				++hit_index;
			}

			for (uint32 i = 0; i < physx_buffer_out->getNbTouches() && hit_index < result_buffer_size; ++i)
			{
				create_location_hit(physx_buffer_out->touches[i], SceneQueryHitType::TOUCH, result_buffer_out[hit_index]);
				++hit_index;
			}
		}

		return hit_index;
	}

	static uint32 sweep_internal(
		const World* world,
		const physx::PxGeometry& geometry,
		const trs& geometry_transform,
		const vec3& direction,
		float distance,
		bool any,
		bool use_single_sided_mesh_backface_hits,
		SceneQueryFilterFlag flags,
		SceneQueryFilterCallback* callback,
		physx::PxSweepBuffer* physx_buffer_out,
		uint32 result_buffer_size,
		SceneQueryPositionedHit* result_buffer_out)
	{
		using namespace physx;

		ASSERT(world != nullptr);
		ASSERT(distance > 0.0f);

		ASSERT(((flags & SceneQueryFilterFlag::STATICS) == SceneQueryFilterFlag::STATICS) ||
			((flags & SceneQueryFilterFlag::DYNAMICS) == SceneQueryFilterFlag::DYNAMICS));

		if (callback != nullptr)
		{
			ASSERT(((flags & SceneQueryFilterFlag::PREFILTER) == SceneQueryFilterFlag::PREFILTER) ||
				((flags & SceneQueryFilterFlag::POSTFILTER) == SceneQueryFilterFlag::POSTFILTER));
		}

		ASSERT(result_buffer_size > 0);
		ASSERT(result_buffer_out != nullptr);

		ref<Physics> physics = PhysicsHolder::physics_ref;

		const PxScene* physx_scene = physics->get_scene();

		if (physx_scene == nullptr)
		{
			return 0;
		}

		uint32 physx_query_flags = create_phyx_query_flags(any, flags);

		PxHitFlags hit_flags = create_physx_hit_flags(use_single_sided_mesh_backface_hits, result_buffer_size);

		PxQueryFilterData physx_filter_data(static_cast<PxQueryFlag::Enum>(physx_query_flags));

		const vec3 normalized_direction = normalize(direction);
		PhysXQueryFilterCallback physx_proxy_callback(SceneQueryType::SWEEP, callback);
		const bool hit = physx_scene->sweep(
			geometry,
			create_PxTransform(geometry_transform),
			create_PxVec3(normalized_direction),
			distance,
			*physx_buffer_out,
			hit_flags,
			physx_filter_data,
			(callback == nullptr) ? nullptr : &physx_proxy_callback);

		uint32 hit_index = 0;

		if (hit)
		{
			if (physx_buffer_out->hasBlock)
			{
				create_location_hit(physx_buffer_out->block, SceneQueryHitType::BLOCK, result_buffer_out[hit_index]);
				++hit_index;
			}

			for (uint32 i = 0; i < physx_buffer_out->getNbTouches() && hit_index < result_buffer_size; ++i)
			{
				create_location_hit(physx_buffer_out->touches[i], SceneQueryHitType::TOUCH, result_buffer_out[hit_index]);
				++hit_index;
			}
		}

		return hit_index;
	}

	static uint32 overlap_internal(
		const World* world,
		const physx::PxGeometry& geometry,
		const trs& geometry_transform,
		bool any,
		SceneQueryFilterFlag flags,
		SceneQueryFilterCallback* callback,
		physx::PxOverlapBuffer* physx_buffer_out,
		uint32 result_buffer_size,
		SceneQueryHit* result_buffer_out)
	{
		using namespace physx;

		ASSERT(world != nullptr);

		ASSERT(((flags & SceneQueryFilterFlag::STATICS) == SceneQueryFilterFlag::STATICS) ||
			((flags & SceneQueryFilterFlag::DYNAMICS) == SceneQueryFilterFlag::DYNAMICS));

		if (callback != nullptr)
		{
			ASSERT(((flags & SceneQueryFilterFlag::PREFILTER) == SceneQueryFilterFlag::PREFILTER) ||
				((flags & SceneQueryFilterFlag::POSTFILTER) == SceneQueryFilterFlag::POSTFILTER));
		}

		ASSERT(result_buffer_size > 0);
		ASSERT(result_buffer_out != nullptr);

		ref<Physics> physics = PhysicsHolder::physics_ref;
		ASSERT(physics != nullptr);

		const PxScene* physx_scene = physics->get_scene();
		if (physx_scene == nullptr)
		{
			return 0;
		}

		uint32 physx_query_flags = create_phyx_query_flags(any, flags);

		PxQueryFilterData physx_filter_data(static_cast<PxQueryFlag::Enum>(physx_query_flags));

		PhysXQueryFilterCallback physx_proxy_callback(SceneQueryType::OVERLAP, callback);
		const bool hit = physx_scene->overlap(
			geometry,
			create_PxTransform(geometry_transform),
			*physx_buffer_out,
			physx_filter_data,
			(callback == nullptr) ? nullptr : &physx_proxy_callback);

		uint32 hit_index = 0;

		if (hit)
		{
			if (physx_buffer_out->hasBlock)
			{
				result_buffer_out[hit_index].type = SceneQueryHitType::BLOCK;
				++hit_index;
			}

			for (uint32 i = 0; i < physx_buffer_out->getNbTouches() && hit_index < result_buffer_size; ++i)
			{
				result_buffer_out[hit_index].type = SceneQueryHitType::TOUCH;
				++hit_index;
			}
		}
		return hit_index;
	}

	bool RaycastQuery::any(World* world, const RaycastQuery::Params& params, SceneQueryPositionedHit* out_hit)
	{
		using namespace physx;

		PxRaycastBuffer physx_buffer;
		const uint32 hits_count = raycast_internal(
			world, 
			params.query_ray, 
			params.distance,
			true,
			params.use_single_sided_mesh_backface_hits,
			params.filter_flags, 
			params.filter_callback,
			&physx_buffer, 
			1, 
			out_hit);

		return (hits_count == 1);
	}

	bool RaycastQuery::closest(World* world, const RaycastQuery::Params& params, SceneQueryPositionedHit* out_hit)
	{
		using namespace physx;

		PxRaycastBuffer physx_buffer;
		const uint32 hits_count = raycast_internal(
			world, 
			params.query_ray, 
			params.distance,
			false,
			params.use_single_sided_mesh_backface_hits,
			params.filter_flags, 
			params.filter_callback,
			&physx_buffer, 
			1, 
			out_hit);

		return (hits_count == 1);
	}

	uint32 RaycastQuery::all(World* world, const RaycastQuery::Params& params, SceneQueryPositionedHit* out_hit_buffer, uint32 out_hit_buffer_size)
	{
		using namespace physx;

		static const uint32 MAX_TOUCHES_COUNT = 63;
		ASSERT(out_hit_buffer_size <= MAX_TOUCHES_COUNT + 1);

		PxRaycastHit physx_hits[MAX_TOUCHES_COUNT];
		PxRaycastBuffer physx_buffer(physx_hits, out_hit_buffer_size);
		const uint32 hits_count = raycast_internal(
			world,
			params.query_ray,
			params.distance,
			false,
			params.use_single_sided_mesh_backface_hits,
			params.filter_flags, 
			params.filter_callback,
			&physx_buffer, 
			out_hit_buffer_size,
			out_hit_buffer);

		return hits_count;
	}

	bool SweepQuery::any(World* world, const SweepQuery::Params& params, SceneQueryPositionedHit* out_hit)
	{
		using namespace physx;

		PxSweepBuffer physx_buffer;
		uint32 hits_count = 0;

		auto sweep_call = [&](const PxGeometry& physx_geometry) {
			hits_count = sweep_internal(
				world,
				physx_geometry,
				params.geometry_transform,
				params.direction,
				params.distance,
				true,
				params.use_single_sided_mesh_backface_hits,
				params.filter_flags,
				params.filter_callback,
				&physx_buffer,
				1,
				out_hit);
			};

		if (params.geometry.type == SceneQueryGeometry::Type::SPHERE)
		{
			sweep_call(PxSphereGeometry(params.geometry.sphere_radius));
		}
		else if (params.geometry.type == SceneQueryGeometry::Type::BOX)
		{
			sweep_call(PxBoxGeometry(create_PxVec3(params.geometry.box_half_extents)));
		}
		else if (params.geometry.type == SceneQueryGeometry::Type::CAPSULE)
		{
			sweep_call(PxCapsuleGeometry(params.geometry.capsule_radius, params.geometry.capsule_half_height));
		}

		return (hits_count == 1);
	}

	bool SweepQuery::closest(World* world, const SweepQuery::Params& params, SceneQueryPositionedHit* out_hit)
	{
		using namespace physx;

		PxSweepBuffer physx_buffer;
		uint32 hits_count = 0;

		auto sweep_call = [&](const PxGeometry& physx_geometry) {
			hits_count = sweep_internal(
				world,
				physx_geometry,
				params.geometry_transform,
				params.direction,
				params.distance,
				false,
				params.use_single_sided_mesh_backface_hits,
				params.filter_flags,
				params.filter_callback,
				&physx_buffer,
				1,
				out_hit);
			};

		if (params.geometry.type == SceneQueryGeometry::Type::SPHERE)
		{
			sweep_call(PxSphereGeometry(params.geometry.sphere_radius));
		}
		else if (params.geometry.type == SceneQueryGeometry::Type::BOX)
		{
			sweep_call(PxBoxGeometry(create_PxVec3(params.geometry.box_half_extents)));
		}
		else if (params.geometry.type == SceneQueryGeometry::Type::CAPSULE)
		{
			sweep_call(PxCapsuleGeometry(params.geometry.capsule_radius, params.geometry.capsule_half_height));
		}

		return (hits_count == 1);
	}

	uint32 SweepQuery::all(World* world, const SweepQuery::Params& params, SceneQueryPositionedHit* out_hit_buffer, uint32 out_hit_buffer_size)
	{
		using namespace physx;

		static const uint32 MAX_TOUCHES_COUNT = 63;
		ASSERT(out_hit_buffer_size <= MAX_TOUCHES_COUNT + 1);

		PxSweepHit physx_hits[MAX_TOUCHES_COUNT];
		PxSweepBuffer physx_buffer(physx_hits, out_hit_buffer_size);

		uint32 hits_count = 0;

		auto sweep_call = [&](const PxGeometry& physx_geometry) {
			hits_count = sweep_internal(
				world,
				physx_geometry,
				params.geometry_transform,
				params.direction,
				params.distance,
				false,
				params.use_single_sided_mesh_backface_hits,
				params.filter_flags,
				params.filter_callback,
				&physx_buffer,
				out_hit_buffer_size,
				out_hit_buffer);
			};

		if (params.geometry.type == SceneQueryGeometry::Type::SPHERE)
		{
			sweep_call(PxSphereGeometry(params.geometry.sphere_radius));
		}
		else if (params.geometry.type == SceneQueryGeometry::Type::BOX)
		{
			sweep_call(PxBoxGeometry(create_PxVec3(params.geometry.box_half_extents)));
		}
		else if (params.geometry.type == SceneQueryGeometry::Type::CAPSULE)
		{
			sweep_call(PxCapsuleGeometry(params.geometry.capsule_radius, params.geometry.capsule_half_height));
		}

		return hits_count;
	}

	bool OverlapQuery::any(World* world, const OverlapQuery::Params& params, SceneQueryPositionedHit* out_hit)
	{
		using namespace physx;

		PxOverlapBuffer physx_buffer;
		uint32 hits_count = 0;

		auto overlap_call = [&](const PxGeometry& physx_geometry) {
			hits_count = overlap_internal(
				world,
				physx_geometry,
				params.geometry_transform,
				true,
				params.filter_flags,
				params.filter_callback,
				&physx_buffer,
				1,
				out_hit);
			};

		if (params.geometry.type == SceneQueryGeometry::Type::SPHERE)
		{
			overlap_call(PxSphereGeometry(params.geometry.sphere_radius));
		}
		else if (params.geometry.type == SceneQueryGeometry::Type::BOX)
		{
			overlap_call(PxBoxGeometry(create_PxVec3(params.geometry.box_half_extents)));
		}
		else if (params.geometry.type == SceneQueryGeometry::Type::CAPSULE)
		{
			overlap_call(PxCapsuleGeometry(params.geometry.capsule_radius, params.geometry.capsule_half_height));
		}

		return (hits_count == 1);
	}

	bool OverlapQuery::closest(const ShapeComponent* shape, const SceneQueryGeometry& geom, const trs& geom_pose)
	{
		using namespace physx;

		ASSERT(shape != nullptr);

		const PxShape* px_shape = shape->get_shape();
		if (px_shape == nullptr)
		{
			return false;
		}

		const PxRigidActor* actor = px_shape->getActor();
		if (actor == nullptr)
		{
			return false;
		}

		PxTransform shape_pose = actor->getGlobalPose();
		shape_pose = shape_pose * px_shape->getLocalPose();

		PxGeometryHolder shape_geometry = px_shape->getGeometry();

		PxTransform pose = create_PxTransform(geom_pose);
		auto overlap_call = [&](const PxGeometry& physx_geometry) {
			return PxGeometryQuery::overlap(shape_geometry.any(), shape_pose, physx_geometry, pose);
			};

		if (geom.type == SceneQueryGeometry::Type::SPHERE)
		{
			return overlap_call(PxSphereGeometry(geom.sphere_radius));
		}
		else if (geom.type == SceneQueryGeometry::Type::BOX)
		{
			return overlap_call(PxBoxGeometry(create_PxVec3(geom.box_half_extents)));
		}
		else if (geom.type == SceneQueryGeometry::Type::CAPSULE)
		{
			return overlap_call(PxCapsuleGeometry(geom.capsule_radius, geom.capsule_half_height));
		}

		return false;
	}

	uint32 OverlapQuery::all(World* world, const OverlapQuery::Params& params, SceneQueryPositionedHit* out_hit_buffer, uint32 out_hit_buffer_size)
	{
		using namespace physx;

		static const uint32 MAX_TOUCHES_COUNT = 256;
		ASSERT(out_hit_buffer_size <= MAX_TOUCHES_COUNT + 1);

		PxOverlapHit physx_hits[MAX_TOUCHES_COUNT];
		PxOverlapBuffer physx_buffer(physx_hits, out_hit_buffer_size);

		uint32 hits_count = 0;

		auto overlap_call = [&](const PxGeometry& physx_geometry) {
			hits_count = overlap_internal(
				world,
				physx_geometry,
				params.geometry_transform,
				false,
				params.filter_flags,
				params.filter_callback,
				&physx_buffer,
				out_hit_buffer_size,
				out_hit_buffer);
			};

		if (params.geometry.type == SceneQueryGeometry::Type::SPHERE)
		{
			overlap_call(PxSphereGeometry(params.geometry.sphere_radius));
		}
		else if (params.geometry.type == SceneQueryGeometry::Type::BOX)
		{
			overlap_call(PxBoxGeometry(create_PxVec3(params.geometry.box_half_extents)));
		}
		else if (params.geometry.type == SceneQueryGeometry::Type::CAPSULE)
		{
			overlap_call(PxCapsuleGeometry(params.geometry.capsule_radius, params.geometry.capsule_half_height));
		}

		return hits_count;
	}

	SceneQueryFilter::SceneQueryFilter(uint32 _collision_mask, uint32 _ignore_mask)
		: collision_mask(_collision_mask)
		, ignore_mask(_ignore_mask)
	{

	}

	SceneQueryFilter::SceneQueryFilter(uint32 _collision_mask, uint32 _ignore_mask, EntityPtr _ignore_entity)
		: SceneQueryFilter(_collision_mask, _ignore_mask)
	{
		ignore_list.push_back(_ignore_entity);
	}

	SceneQueryFilter::SceneQueryFilter(uint32 _collision_mask, uint32 _ignore_mask, const std::vector<EntityPtr>& _ignore_list)
		: collision_mask(_collision_mask)
		, ignore_mask(_ignore_mask)
		, ignore_list(_ignore_list)
	{

	}

	SceneQueryHitType SceneQueryFilter::prefilter(const ShapeComponent* component)
	{
		if ((static_cast<uint32>(component->collision_type.get()) & collision_mask) > 0 &&
			(static_cast<uint32>(component->collision_type.get()) & ignore_mask) == 0)
		{
			Entity entity = component->get_entity();
			for (const EntityPtr& entity_ptr_to_ignore : ignore_list)
			{
				if (entity_ptr_to_ignore.is_empty() ||
					entity == entity_ptr_to_ignore.get())
				{
					return SceneQueryHitType::NONE;
				}
			}

			return SceneQueryHitType::BLOCK;
		}
		else
		{
			return SceneQueryHitType::NONE;
		}
	}

	SceneQueryFilter SceneQueryFilter::construct_from_type(World* world, uint32 collision_type)
	{
		const CollisionsHolderRootComponent* collision_holder_rc = world->get_root_component<CollisionsHolderRootComponent>();

		SceneQueryFilter filter;
		filter.collision_mask = collision_holder_rc->get_collision_filter(collision_type);
		return filter;
	}
}