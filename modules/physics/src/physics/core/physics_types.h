#pragma once

#include "physics/physx_api.h"
#include "physics_api.h"

#include <ecs/component.h>
#include <ecs/entity.h>

namespace era_engine::physics
{
	class PhysicsAllocatorCallback : public physx::PxAllocatorCallback
	{
	public:
		void* allocate(size_t size, const char* type_name, const char* filename, int line) override;

		void deallocate(void* ptr) override;
	};

	template<typename HitType>
	class DynamicHitBuffer : public physx::PxHitCallback<HitType>
	{
	public:
		DynamicHitBuffer() : physx::PxHitCallback<HitType>(buffer, PX_CONTACT_BUFFER_SIZE), count(0)
		{
		}

	public:
		PX_INLINE physx::PxU32 getNbAnyHits() const
		{
			return getNbTouches();
		}

		PX_INLINE const HitType& getAnyHit(const physx::PxU32 index) const
		{
			PX_ASSERT(index < getNbTouches() + physx::PxU32(this->hasBlock));
			return index < getNbTouches() ? getTouches()[index] : this->block;
		}

		PX_INLINE physx::PxU32 getNbTouches() const
		{
			return count;
		}

		PX_INLINE const HitType* getTouches() const
		{
			return buffer;
		}

		PX_INLINE const HitType& getTouch(const physx::PxU32 index) const
		{
			PX_ASSERT(index < getNbTouches());
			return buffer[index];
		}

		PX_INLINE physx::PxU32 getMaxNbTouches() const
		{
			return PX_CONTACT_BUFFER_SIZE;
		}

	protected:
		physx::PxAgain processTouches(const HitType* in_buffer, physx::PxU32 nb_hits) override
		{
			using namespace physx;
			nb_hits = min(nb_hits, PX_CONTACT_BUFFER_SIZE - count);
			for (PxU32 i = 0; i < nb_hits; i++)
			{
				buffer[count + i] = in_buffer[i];
			}
			count += nb_hits;
			return true;
		}

		void finalizeQuery() override
		{
			if (this->hasBlock)
			{
				processTouches(&this->block, 1);
			}
		}

	private:
		uint32_t count;
		HitType buffer[PX_CONTACT_BUFFER_SIZE];
	};

	class OverlapCallback : public physx::PxOverlapBufferN<32>
	{
	public:
		OverlapCallback() = default;

		physx::PxAgain processTouches(const physx::PxOverlapHit* buffer, physx::PxU32 nb_hits);

		physx::PxRigidDynamic* hit_actor = nullptr;
	};

	class SimulationFilterCallback : public physx::PxSimulationFilterCallback
	{
	public:
		physx::PxFilterFlags pairFound(physx::PxU64 pairID,
			physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0, const physx::PxActor* a0, const physx::PxShape* s0,
			physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1, const physx::PxActor* a1, const physx::PxShape* s1,
			physx::PxPairFlags& pairFlags) override;

		void pairLost(physx::PxU64 pairID,
			physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
			physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
			bool objectRemoved) override;

		bool statusChange(physx::PxU64& pairID, physx::PxPairFlags& pairFlags, physx::PxFilterFlags& filterFlags) override;
	};

	class CharacterControllerHitReportCallback final : public physx::PxUserControllerHitReport
	{
	public:
		CharacterControllerHitReportCallback(World* world_);

		void onShapeHit(const physx::PxControllerShapeHit& hit) override;

		void onControllerHit(const physx::PxControllersHit& hit) override;

		void onObstacleHit(const physx::PxControllerObstacleHit& hit) override;

	private:
		World* world = nullptr;
	};

	class ProfilerCallback : public physx::PxProfilerCallback
	{
	public:
		void* zoneStart(const char* eventName, bool detached, uint64_t contextId) override;

		void zoneEnd(void* profilerData, const char* eventName, bool detached, uint64_t contextId) override;
	};

	class ErrorReporter : public physx::PxErrorCallback
	{
	public:
		void reportError(physx::PxErrorCode::Enum code, const char* message, const char* file, int line) override;
	};

	class BodyComponent;
	class CharacterControllerComponent;

	struct ERA_PHYSICS_API CollisionHandlingData
	{
		Entity::Handle id1;
		Entity::Handle id2;
	};

	struct ERA_PHYSICS_API ContactPoint
	{
		physx::PxVec3 point;

		physx::PxVec3 normal;

		float separation;
	};

	class ERA_PHYSICS_API Collision final
	{
	public:
		physx::PxVec3 get_relative_velocity() const;
		void swap_objects();

	public:
		BodyComponent* this_actor = nullptr;
		BodyComponent* other_actor = nullptr;

		physx::PxVec3 impulse;
		physx::PxVec3 this_velocity;
		physx::PxVec3 other_velocity;

		int32 contacts_count;

		ContactPoint contacts[PX_CONTACT_BUFFER_SIZE];
	};

	struct ERA_PHYSICS_API CharacterControllerCollision final
	{
		CharacterControllerComponent* character_controller = nullptr;

		ComponentPtr other_ptr;

		vec3 position;
		vec3 normal;
		vec3 direction;

		float length = 0.f;
	};

	class ExplodeOverlapCallback : public physx::PxOverlapCallback
	{
	public:
		ExplodeOverlapCallback(const physx::PxVec3& _world_pos, float _radius, float _explosive_impulse)
			: world_position(_world_pos)
			, radius(_radius)
			, explosive_impulse(_explosive_impulse)
			, physx::PxOverlapCallback(hit_buffer, sizeof(hit_buffer) / sizeof(hit_buffer[0])) {}

		physx::PxAgain processTouches(const physx::PxOverlapHit* buffer, physx::PxU32 nbHits);

	private:
		std::set<physx::PxRigidDynamic*> actor_buffer;
		float explosive_impulse = 1.0f;
		float radius = 1.0f;
		physx::PxVec3 world_position;
		physx::PxOverlapHit hit_buffer[512];
	};

	class JointComponent;

	class SimulationEventCallback : public physx::PxSimulationEventCallback
	{
	public:
		SimulationEventCallback() = default;

		typedef std::pair<BodyComponent*, BodyComponent*> CollidersPair;

		void clear();

		void sendCollisionEvents();

		void sendTriggerEvents();

		void sendJointEvents();

		void onColliderRemoved(BodyComponent* collider);

		void onJointRemoved(JointComponent* joint);

		void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count) override;
		void onWake(physx::PxActor** actors, physx::PxU32 count) override;
		void onSleep(physx::PxActor** actors, physx::PxU32 count) override;
		void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count) override;
		void onAdvance(const physx::PxRigidBody* const* body_buffer, const physx::PxTransform* pose_buffer, const physx::PxU32 count) override;
		void onContact(const physx::PxContactPairHeader& pair_header, const physx::PxContactPair* pairs, physx::PxU32 nb_pairs) override;

	public:
		physx::PxArray<Collision> new_collisions;
		physx::PxArray<CharacterControllerCollision> cct_collisions;

		physx::PxArray<JointComponent*> broken_joints;

		physx::PxArray<Collision> removed_collisions;

		physx::PxArray<CollidersPair> new_trigger_pairs;

		physx::PxArray<CollidersPair> lost_trigger_pairs;
	};

}