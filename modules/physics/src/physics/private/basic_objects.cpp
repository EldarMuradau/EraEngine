#include "physics/basic_objects.h"
#include "physics/core/physics.h"
#include "physics/shape_utils.h"
#include "physics/shape_component.h"

#include <terrain/terrain.h>

#include <rttr/registration>

namespace era_engine::physics
{

	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<PlanePhysicsComponent>("PlanePhysicsComponent")
			.constructor<>();
	}

	PlanePhysicsComponent::PlanePhysicsComponent(ref<Entity::EcsData> _data, CollisionType _collision_type, const vec3& _point, const vec3& _norm)
		: Component(_data), point(_point), normal(_norm)
	{
		using namespace physx;

		auto physics = PhysicsEngine::get_physics_core();

		plane = PxCreatePlane(*physics->get_physics(), PxPlane(create_PxVec3(point), create_PxVec3(normal)), *physics->get_default_material()->get_native_material());

		PxShape* buffer[1];
		plane->getShapes(buffer, 1);
		ShapeUtils::setup_filtering(get_world(), buffer[0], static_cast<uint32>(_collision_type), std::nullopt);

		PhysicsEngine::execute_write([&]() {
			physics->get_scene()->addActor(*plane);
		});
	}

	PlanePhysicsComponent::~PlanePhysicsComponent()
	{
		using namespace physx;

		auto physics = PhysicsEngine::get_physics_core();

		PxShape* buffer[1];
		plane->getShapes(buffer, 1);

		PhysicsEngine::execute_write([&]() {
			plane->detachShape(*buffer[0]);

			physics->get_scene()->removeActor(*plane);

			PX_RELEASE(buffer[0])
			PX_RELEASE(plane)
			});
	}

	TerrainPhysicsComponent::TerrainPhysicsComponent(ref<Entity::EcsData> _data, CollisionType _collision_type, const vec3& _point)
		: Component(_data), point(_point)
	{
		using namespace physx;

		auto physics = PhysicsEngine::get_physics_core();

		const TerrainComponent* terrain_component = get_entity().get_component<TerrainComponent>();
		ASSERT(terrain_component != nullptr);

		terrain = physics->get_physics()->createRigidStatic(PxTransform(create_PxVec3(_point)));

		const uint32 ts = terrain_component->chunksPerDim;
		const size_t verts_per_chunk = terrain_component->chunk(0).heights.size();

		PxReal min_height = PX_MAX_F32;
		PxReal max_height = -PX_MAX_F32;
		for (PxU32 s = 0; s < ts * ts; s++)
		{
			const TerrainChunk& chunk = terrain_component->chunk(s);

			for (size_t i = 0; i < verts_per_chunk; ++i)
			{
				const PxReal height = PxReal(chunk.heights[i]);
				min_height = PxMin(min_height, height);
				max_height = PxMax(max_height, height);
			}
		}

		PxReal delta_height = max_height - min_height;

		// Maximum positive value that can be represented with signed 16 bit integer.
		PxReal quantization = (PxReal)0x7fff;

		const size_t number_of_vertices_per_chunk = ts * verts_per_chunk;

		// Compute heightScale such that the forward transform will generate the closest point to the source,
		// clamp to at least PX_MIN_HEIGHTFIELD_Y_SCALE to respect the PhysX API specs.
		PxReal height_scale = PxMax(delta_height / quantization, PX_MIN_HEIGHTFIELD_Y_SCALE);

		PxHeightFieldSample* hf_samples = new PxHeightFieldSample[number_of_vertices_per_chunk * number_of_vertices_per_chunk];

		constexpr bool flip_edge = false;
		for (PxU32 s = 0; s < ts * ts; s++)
		{
			const TerrainChunk& chunk = terrain_component->chunk(s);

			for (size_t i = 0; i < verts_per_chunk; ++i)
			{
				const PxReal raw_height = PxReal(chunk.heights[i]);

				PxI16 height = PxI16(quantization * ((raw_height - min_height) / delta_height));

				PxHeightFieldSample& smp = hf_samples[(verts_per_chunk * s) + i];
				smp.height = height;
				smp.materialIndex0 = 0;
				smp.materialIndex1 = 0;
				if (flip_edge)
				{
					smp.setTessFlag();
				}
			}
		}

		PxHeightFieldDesc terrain_desc;
		terrain_desc.format = PxHeightFieldFormat::eS16_TM;
		terrain_desc.nbColumns = number_of_vertices_per_chunk;
		terrain_desc.nbRows = number_of_vertices_per_chunk;
		terrain_desc.samples.data = hf_samples;
		terrain_desc.samples.stride = sizeof(PxHeightFieldSample);
		terrain_desc.flags = PxHeightFieldFlags();

		const float terrain_width = terrain_component->chunkSize * number_of_vertices_per_chunk;

		PxHeightFieldGeometry hf_geom;
		hf_geom.columnScale = terrain_width / (ts - 1);
		hf_geom.rowScale = terrain_width / (ts - 1);
		hf_geom.heightScale = delta_height != 0.0f ? height_scale : 1.0f;
		hf_geom.heightField = PxCreateHeightField(terrain_desc, physics->get_physics()->getPhysicsInsertionCallback());

		delete[] hf_samples;

		PxTransform local_pose;
		local_pose.p = PxVec3(-(terrain_width * 0.5f), min_height, -(terrain_width * 0.5f));
		local_pose.q = PxQuat(PxIdentity);

		const ref<PhysicsMaterial>& used_material = material == nullptr ? physics->get_default_material() : material;

		PxMaterial* materials[1];
		materials[0] = used_material->get_native_material();

		shape = PxRigidActorExt::createExclusiveShape(*terrain, hf_geom, materials, 1);
		shape->setLocalPose(local_pose);

		ShapeUtils::enable_shape_visualization(shape, false);

		ShapeUtils::setup_filtering(get_world(), shape, static_cast<uint32>(_collision_type), std::nullopt);

		PhysicsEngine::execute_write([&]() {
			physics->get_scene()->addActor(*terrain);
			});
	}

	TerrainPhysicsComponent::~TerrainPhysicsComponent()
	{
		using namespace physx;

		auto physics = PhysicsEngine::get_physics_core();

		PhysicsEngine::execute_write([&]() {
			terrain->detachShape(*shape);

			physics->get_scene()->removeActor(*terrain);

			PX_RELEASE(shape)
			PX_RELEASE(terrain)
			});
	}

	physx::PxRigidDynamic* create_rigid_cube(physx::PxReal half_extent, const physx::PxVec3& position)
	{
		using namespace physx;
		auto* physics = PhysicsEngine::get_physics_core()->get_physics();

		PxMaterial* material = physics->createMaterial(0.8f, 0.8f, 0.6f);

		PxShape* shape = physics->createShape(PxBoxGeometry(half_extent, half_extent, half_extent), *material);

		shape->setSimulationFilterData(PxFilterData(0, 0, 1, 0));

		PxTransform localTm(position);
		PxRigidDynamic* body = physics->createRigidDynamic(localTm);
		body->attachShape(*shape);
		PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);

		shape->release();

		return body;
	}

	void create_cube(physx::PxArray<physx::PxVec3>& tri_verts, physx::PxArray<physx::PxU32>& tri_indices, const physx::PxVec3& pos, physx::PxReal scaling)
	{
		using namespace physx;

		tri_verts.clear();
		tri_indices.clear();

		tri_verts.pushBack(scaling * PxVec3(0.5f, -0.5f, -0.5f) + pos);
		tri_verts.pushBack(scaling * PxVec3(0.5f, -0.5f, 0.5f) + pos);
		tri_verts.pushBack(scaling * PxVec3(-0.5f, -0.5f, 0.5f) + pos);
		tri_verts.pushBack(scaling * PxVec3(-0.5f, -0.5f, -0.5f) + pos);
		tri_verts.pushBack(scaling * PxVec3(0.5f, 0.5f, -0.5f) + pos);
		tri_verts.pushBack(scaling * PxVec3(0.5f, 0.5f, 0.5f) + pos);
		tri_verts.pushBack(scaling * PxVec3(-0.5f, 0.5f, 0.5f) + pos);
		tri_verts.pushBack(scaling * PxVec3(-0.5f, 0.5f, -0.5f) + pos);

		tri_indices.pushBack(1); tri_indices.pushBack(2); tri_indices.pushBack(3);
		tri_indices.pushBack(7); tri_indices.pushBack(6); tri_indices.pushBack(5);
		tri_indices.pushBack(4); tri_indices.pushBack(5); tri_indices.pushBack(1);
		tri_indices.pushBack(5); tri_indices.pushBack(6); tri_indices.pushBack(2);

		tri_indices.pushBack(2); tri_indices.pushBack(6); tri_indices.pushBack(7);
		tri_indices.pushBack(0); tri_indices.pushBack(3); tri_indices.pushBack(7);
		tri_indices.pushBack(0); tri_indices.pushBack(1); tri_indices.pushBack(3);
		tri_indices.pushBack(4); tri_indices.pushBack(7); tri_indices.pushBack(5);

		tri_indices.pushBack(0); tri_indices.pushBack(4); tri_indices.pushBack(1);
		tri_indices.pushBack(1); tri_indices.pushBack(5); tri_indices.pushBack(2);
		tri_indices.pushBack(3); tri_indices.pushBack(2); tri_indices.pushBack(7);
		tri_indices.pushBack(4); tri_indices.pushBack(0); tri_indices.pushBack(7);	
	}
}