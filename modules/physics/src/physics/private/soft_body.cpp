#include "physics/soft_body.h"
#include "physics/core/physics.h"

#include "ecs/base_components/transform_component.h"

#include <rttr/registration>

namespace era_engine::physics
{

	RTTR_REGISTRATION
	{
		using namespace rttr;
		rttr::registration::class_<SoftBodyComponent>("SoftBodyComponent")
			.constructor<>();
	}

	static void process_soft_body(physx::PxSoftBody* softBody, const physx::PxFEMParameters& femParams, const physx::PxTransform& transform, const physx::PxReal density, const physx::PxReal scale, const physx::PxU32 iterCount)
	{
		using namespace physx;
		PxVec4* simPositionInvMassPinned;
		PxVec4* simVelocityPinned;
		PxVec4* collPositionInvMassPinned;
		PxVec4* restPositionPinned;

		PxCudaContextManager* context_managert = PhysicsEngine::get_physics_core()->get_cuda_context_manager();

		PxSoftBodyExt::allocateAndInitializeHostMirror(*softBody, context_managert, simPositionInvMassPinned, simVelocityPinned, collPositionInvMassPinned, restPositionPinned);

		const PxReal maxInvMassRatio = 50.f;

		softBody->setParameter(femParams);
		softBody->setSolverIterationCounts(iterCount);

		PxSoftBodyExt::transform(*softBody, transform, scale, simPositionInvMassPinned, simVelocityPinned, collPositionInvMassPinned, restPositionPinned);
		PxSoftBodyExt::updateMass(*softBody, density, maxInvMassRatio, simPositionInvMassPinned);
		PxSoftBodyExt::copyToDevice(*softBody, PxSoftBodyDataFlag::eALL, simPositionInvMassPinned, simVelocityPinned, collPositionInvMassPinned, restPositionPinned);

		PX_PINNED_HOST_FREE(context_managert, simPositionInvMassPinned);
		PX_PINNED_HOST_FREE(context_managert, simVelocityPinned);
		PX_PINNED_HOST_FREE(context_managert, collPositionInvMassPinned);
		PX_PINNED_HOST_FREE(context_managert, restPositionPinned);
	}

	void connect_cube_to_soft_body(physx::PxRigidDynamic* cube, physx::PxReal cube_half_extent, const physx::PxVec3& cube_position, physx::PxSoftBody* soft_body, physx::PxU32 point_grid_resolution)
	{
		using namespace physx;

		float f = 2.0f * cube_half_extent / (point_grid_resolution - 1);
		for (PxU32 ix = 0; ix < point_grid_resolution; ++ix)
		{
			PxReal x = ix * f - cube_half_extent;
			for (PxU32 iy = 0; iy < point_grid_resolution; ++iy)
			{
				PxReal y = iy * f - cube_half_extent;
				for (PxU32 iz = 0; iz < point_grid_resolution; ++iz)
				{
					PxReal z = iz * f - cube_half_extent;
					PxVec3 p(x, y, z);
					PxVec4 bary;
					PxI32 tet = PxTetrahedronMeshExt::findTetrahedronContainingPoint(soft_body->getCollisionMesh(), p + cube_position, bary);
					if (tet >= 0)
					{
						soft_body->addTetRigidAttachment(cube, tet, bary, p);
					}
				}
			}
		}
	}

	SoftBodyComponent::SoftBodyComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
		using namespace physx;
		auto physics_core = PhysicsEngine::get_physics_core();
		// Test
		{
			PxArray<PxVec3> triVerts;
			PxArray<PxU32> triIndices;

			PxReal maxEdgeLength = 1;

			PxCookingParams params(physics_core->get_tolerance_scale());
			params.meshWeldTolerance = 0.001f;
			params.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
			params.buildTriangleAdjacencies = false;
			params.buildGPUData = true;

			create_cube(triVerts, triIndices, PxVec3(0, 0, 0), 1.0f);
			PxRemeshingExt::limitMaxEdgeLength(triIndices, triVerts, maxEdgeLength);
			create_soft_body(params, triVerts, triIndices);

			PxReal halfExtent = 1;
			PxVec3 cubePosA(0, 20, 0);
			PxRigidDynamic* body = create_rigid_cube(halfExtent, cubePosA);

			connect_cube_to_soft_body(body, 2 * halfExtent, cubePosA, soft_body);

			physics_core->get_scene()->addActor(*soft_body);
		}

		positions_inv_mass = PX_PINNED_HOST_ALLOC_T(physx::PxVec4, PhysicsEngine::get_physics_core()->get_cuda_context_manager(), soft_body->getCollisionMesh()->getNbVertices());
	}

	SoftBodyComponent::~SoftBodyComponent()
	{
		PhysicsEngine::get_physics_core()->get_scene()->removeActor(*soft_body);

		if (positions_inv_mass)
		{
			PX_PINNED_HOST_FREE(PhysicsEngine::get_physics_core()->get_cuda_context_manager(), positions_inv_mass);
		}
		PX_RELEASE(soft_body)
	}

	uint32 SoftBodyComponent::get_nb_vertices() const
	{
		return soft_body->getCollisionMesh()->getNbVertices();
	}

	void SoftBodyComponent::create_soft_body(const physx::PxCookingParams& params, const physx::PxArray<physx::PxVec3>& tri_verts, const physx::PxArray<physx::PxU32>& tri_indices, bool use_collision_mesh_for_simulation)
	{
		using namespace physx;

		const auto physics = PhysicsEngine::get_physics_core()->get_physics();

		PxSoftBodyMesh* softBodyMesh;

		PxU32 numVoxelsAlongLongestAABBAxis = 8;

		PxSimpleTriangleMesh surfaceMesh;
		surfaceMesh.points.count = tri_verts.size();
		surfaceMesh.points.data = tri_verts.begin();
		surfaceMesh.triangles.count = tri_indices.size() / 3;
		surfaceMesh.triangles.data = tri_indices.begin();

		if (use_collision_mesh_for_simulation)
		{
			softBodyMesh = PxSoftBodyExt::createSoftBodyMeshNoVoxels(params, surfaceMesh, physics->getPhysicsInsertionCallback());
		}
		else
		{
			softBodyMesh = PxSoftBodyExt::createSoftBodyMesh(params, surfaceMesh, numVoxelsAlongLongestAABBAxis, physics->getPhysicsInsertionCallback());
		}

		//Alternatively one can cook a softbody mesh in a single step
		//tetMesh = cooking.createSoftBodyMesh(simulationMeshDesc, collisionMeshDesc, softbodyDesc, physics.getPhysicsInsertionCallback());
		PX_ASSERT(softBodyMesh);

		PxCudaContextManager* context_managert = PhysicsEngine::get_physics_core()->get_cuda_context_manager();

		if (!context_managert)
		{
			return;
		}
		soft_body = physics->createSoftBody(*context_managert);
		if (soft_body)
		{
			PxSceneWriteLock _lock(*PhysicsEngine::get_physics_core()->get_scene());

			PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;

			PxFEMSoftBodyMaterial* materialPtr = PxGetPhysics().createFEMSoftBodyMaterial(1e+6f, 0.45f, 0.5f);
			PxTetrahedronMeshGeometry geometry(softBodyMesh->getCollisionMesh());
			PxShape* shape = physics->createShape(geometry, &materialPtr, 1, true, shapeFlags);
			if (shape)
			{
				soft_body->attachShape(*shape);
				shape->setSimulationFilterData(PxFilterData(0, 0, 2, 0));
			}
			soft_body->attachSimulationMesh(*softBodyMesh->getSimulationMesh(), *softBodyMesh->getSoftBodyAuxData());

			const trs& world_transform = get_entity().get_component<TransformComponent>()->get_world_transform();

			PxFEMParameters femParams;
			process_soft_body(soft_body, femParams, create_PxTransform(world_transform), 100.f, 1.0f, 30);
			soft_body->setSoftBodyFlag(PxSoftBodyFlag::eDISABLE_SELF_COLLISION, false);
			soft_body->setSoftBodyFlag(PxSoftBodyFlag::eENABLE_CCD, true);
		}
	}

	void SoftBodyComponent::copy_deformed_vertices_from_gpu_async(CUstream stream)
	{
		using namespace physx;
		PxTetrahedronMesh* tetMesh = soft_body->getCollisionMesh();

		PxCudaContextManager* context_managert = PhysicsEngine::get_physics_core()->get_cuda_context_manager();

		PxScopedCudaLock _lock(*context_managert);
		context_managert->getCudaContext()->memcpyDtoHAsync(positions_inv_mass, reinterpret_cast<CUdeviceptr>(soft_body->getPositionInvMassBufferD()), tetMesh->getNbVertices() * sizeof(PxVec4), stream);
	}

	void SoftBodyComponent::copy_deformed_vertices_from_gpu()
	{
		using namespace physx;
		PxTetrahedronMesh* tetMesh = soft_body->getCollisionMesh();

		PxCudaContextManager* context_managert = PhysicsEngine::get_physics_core()->get_cuda_context_manager();

		PxScopedCudaLock _lock(*context_managert);
		context_managert->getCudaContext()->memcpyDtoH(positions_inv_mass, reinterpret_cast<CUdeviceptr>(soft_body->getPositionInvMassBufferD()), tetMesh->getNbVertices() * sizeof(PxVec4));
	}

}