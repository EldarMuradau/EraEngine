#pragma once

#ifdef setBit
#undef setBit
#endif // setBit

#ifndef PX_PHYSX_STATIC_LIB
#define PX_PHYSX_STATIC_LIB
#endif

#include <PxPhysics.h>
#include <PxPhysicsAPI.h>

#include <extensions/PxRaycastCCD.h>
#include <extensions/PxRemeshingExt.h>
#include <extensions/PxSoftBodyExt.h>
#include <extensions/PxParticleExt.h>
#include <extensions/PxParticleClothCooker.h>

#include <PxPBDParticleSystem.h>

#include <cudamanager/PxCudaContextManager.h>
#include <cudamanager/PxCudaContext.h>

#define PX_CONTACT_BUFFER_SIZE 64

#define PX_ENABLE_RAYCAST_CCD 0

#define PX_PARTICLE_USE_ALLOCATOR 1

#define PX_PHYSICS_ENABLED 1

#define PX_NB_MAX_RAYCAST_HITS 64
#define PX_NB_MAX_RAYCAST_DISTANCE 128

#define PX_VEHICLE 1

#if DEBUG || _DEBUG
#define PX_BLAST_ENABLE 0
#define PX_VEHICLE_DEBUG 1
#else
#define PX_BLAST_ENABLE 1
#define PX_VEHICLE_DEBUG 0
#endif

#ifndef PX_RELEASE
#define PX_RELEASE(x)	if(x)	{ x->release(); x = nullptr;}
#endif

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

#define PX_DEVICE_ALLOC(cudaContextManager, deviceBuffer, numElements) cudaContextManager->allocDeviceBuffer(deviceBuffer, numElements, PX_FL)
#define PX_DEVICE_ALLOC_T(T, cudaContextManager, numElements) cudaContextManager->allocDeviceBuffer<T>(numElements, PX_FL)
#define PX_DEVICE_FREE(cudaContextManager, deviceBuffer) cudaContextManager->freeDeviceBuffer(deviceBuffer);

#define PX_PINNED_HOST_ALLOC(cudaContextManager, pinnedHostBuffer, numElements) cudaContextManager->allocPinnedHostBuffer(pinnedHostBuffer, numElements, PX_FL)
#define PX_PINNED_HOST_ALLOC_T(T, cudaContextManager, numElements) cudaContextManager->allocPinnedHostBuffer<T>(numElements, PX_FL)
#define PX_PINNED_HOST_FREE(cudaContextManager, pinnedHostBuffer) cudaContextManager->freePinnedHostBuffer(pinnedHostBuffer);

#define PX_SCENE_QUERY_SETUP(block_single) \
		const physx::PxHitFlags hitFlags = physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL | physx::PxHitFlag::eMESH_MULTIPLE | physx::PxHitFlag::eUV; \
		physx::PxQueryFilterData filterData; \
		filterData.flags |= physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::eSTATIC; \
		filterData.data.word0 = layer_mask; \
		filterData.data.word1 = block_single ? 1 : 0; \
		filterData.data.word2 = hit_triggers ? 1 : 0

#define PX_SCENE_QUERY_SETUP_SWEEP_CAST_ALL() PX_SCENE_QUERY_SETUP(true); \
		physx::PxSweepBufferN<1> buffer

#define PX_SCENE_QUERY_SETUP_SWEEP_CAST() PX_SCENE_QUERY_SETUP(false); \
		DynamicHitBuffer<physx::PxSweepHit> buffer

#define PX_SCENE_QUERY_SETUP_CHECK() PX_SCENE_QUERY_SETUP(false); \
		physx::PxOverlapBufferN<1> buffer

#define PX_SCENE_QUERY_SETUP_OVERLAP() PX_SCENE_QUERY_SETUP(false); \
		DynamicHitBuffer<physx::PxOverlapHit> buffer

#define PX_SCENE_QUERY_COLLECT_OVERLAP() results.clear(); \
		results.resize(buffer.getNbTouches()); \
		size_t resultSize = results.size(); \
		for (int32 i = 0; i < resultSize; i++) \
		{ \
			const auto& hit = buffer.getTouch(i); \
			results[i] = hit.shape ? *reinterpret_cast<Entity::Handle*>(hit.shape->userData) : Entity::NullHandle; \
		}

#include <core/math.h>
#include <core/math_simd.h>

namespace physx
{
	constexpr float PX_HALF_MAX_F32 = PX_MAX_F32 / 2.0f;

	PX_FORCE_INLINE constexpr PxU32 id(PxU32 x, PxU32 y, PxU32 numY) { return x * numY + y; }

	inline PxVec3 operator*(PxVec3 a, PxVec3 b) { return PxVec3(a.x * b.x, a.y * b.y, a.z * b.z); }
	inline PxVec4 operator*(PxVec4 a, PxVec4 b) { return PxVec4(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w); }

	PX_FORCE_INLINE PxVec4 create_PxVec4(const vec3& vec) { return PxVec4(vec.x, vec.y, vec.z, 0); }
	PX_FORCE_INLINE PxVec4 create_PxVec4(const vec4& vec) { return PxVec4(vec.x, vec.y, vec.z, vec.w); }

	PX_FORCE_INLINE PxExtendedVec3 create_PxExtendedVec3(const vec3& vec) { return PxExtendedVec3(vec.x, vec.y, vec.z); }
	PX_FORCE_INLINE PxVec3 create_PxVec3(const vec3& vec) { return PxVec3(vec.x, vec.y, vec.z); }
	PX_FORCE_INLINE PxVec2 create_PxVec2(const vec2& vec) { return PxVec2(vec.x, vec.y); }
    PX_FORCE_INLINE PxVec3 create_PxVec3(vec3&& vec) { return PxVec3(vec.x, vec.y, vec.z); }
	PX_FORCE_INLINE PxVec2 create_PxVec2(vec2&& vec) { return PxVec2(vec.x, vec.y); }

	PX_FORCE_INLINE PxQuat create_PxQuat(const quat& q) { return PxQuat(q.x, q.y, q.z, q.w); }
	PX_FORCE_INLINE PxQuat create_PxQuat(quat&& q) { return PxQuat(q.x, q.y, q.z, q.w); }

	PX_FORCE_INLINE quat create_quat(const PxQuat& q) { return quat(q.x, q.y, q.z, q.w); }
	PX_FORCE_INLINE quat create_quat(PxQuat&& q) { return quat(q.x, q.y, q.z, q.w); }

	PX_FORCE_INLINE vec3 create_vec3(const PxExtendedVec3& vec) { return vec3(vec.x, vec.y, vec.z); }
	PX_FORCE_INLINE vec3 create_vec3(const PxVec3& vec) { return vec3(vec.x, vec.y, vec.z); }
	PX_FORCE_INLINE vec2 create_vec2(const PxVec2& vec) { return vec2(vec.x, vec.y); }
	PX_FORCE_INLINE vec3 create_vec3(PxVec3&& vec) { return vec3(vec.x, vec.y, vec.z); }
	PX_FORCE_INLINE vec2 create_vec2(PxVec2&& vec) { return vec2(vec.x, vec.y); }

	PX_FORCE_INLINE PxVec2 min(const PxVec2& a, const PxVec2& b) { return PxVec2(std::min(a.x, b.x), std::min(a.y, b.y)); }
	PX_FORCE_INLINE PxVec3 min(const PxVec3& a, const PxVec3& b) { return PxVec3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)); }

	PX_FORCE_INLINE PxVec2 max(const PxVec2& a, const PxVec2& b) { return PxVec2(std::max(a.x, b.x), std::max(a.y, b.y)); }
	PX_FORCE_INLINE PxVec3 max(const PxVec3& a, const PxVec3& b) { return PxVec3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)); }

	PX_FORCE_INLINE trs create_trs(const PxTransform& transform) { return trs(create_vec3(transform.p), create_quat(transform.q), vec3(1.0f)); }
	PX_FORCE_INLINE trs create_trs(PxTransform&& transform) { return trs(create_vec3(transform.p), create_quat(transform.q), vec3(1.0f)); }
	PX_FORCE_INLINE PxTransform create_PxTransform(const trs& transform) { return PxTransform(create_PxVec3(transform.position), create_PxQuat(transform.rotation)); }
	PX_FORCE_INLINE PxTransform create_PxTransform(trs&& transform) { return PxTransform(create_PxVec3(transform.position), create_PxQuat(transform.rotation)); }

	PX_FORCE_INLINE PxMat44 create_PxMat44(const mat4& mat)
	{
		PxMat44 new_mat;

		new_mat[0][0] = mat.m00;
		new_mat[0][1] = mat.m01;
		new_mat[0][2] = mat.m02;
		new_mat[0][3] = mat.m03;
		
		new_mat[1][0] = mat.m10;
		new_mat[1][1] = mat.m11;
		new_mat[1][2] = mat.m12;
		new_mat[1][3] = mat.m13;
		
		new_mat[2][0] = mat.m20;
		new_mat[2][1] = mat.m21;
		new_mat[2][2] = mat.m22;
		new_mat[2][3] = mat.m23;
		
		new_mat[3][0] = mat.m30;
		new_mat[3][1] = mat.m31;
		new_mat[3][2] = mat.m32;
		new_mat[3][3] = mat.m33;

		return new_mat;
	}

	PX_FORCE_INLINE mat4 create_mat44(const PxMat44& mat)
	{
		mat4 new_mat;

		new_mat.m00 = mat[0][0];
		new_mat.m01 = mat[0][1];
		new_mat.m02 = mat[0][2];
		new_mat.m03 = mat[0][3];
		
		new_mat.m10 = mat[1][0];
		new_mat.m11 = mat[1][1];
		new_mat.m12 = mat[1][2];
		new_mat.m13 = mat[1][3];
		
		new_mat.m20 = mat[2][0];
		new_mat.m21 = mat[2][1];
		new_mat.m22 = mat[2][2];
		new_mat.m23 = mat[2][3];
		
		new_mat.m30 = mat[3][0];
		new_mat.m31 = mat[3][1];
		new_mat.m32 = mat[3][2];
		new_mat.m33 = mat[3][3];

		return new_mat;
	}
}

namespace era_engine::physics
{
	static inline physx::PxVec3 PX_GRAVITY(0.0f, -9.81f, 0.0f);
}