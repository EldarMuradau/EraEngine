#include "physics/core/physics_types.h"
#include "physics/core/physics.h"
#include "physics/body_component.h"
#include "physics/collisions_holder_root_component.h"
#include "physics/joint.h"
#include "physics/cct_component.h"

#include "core/memory.h"
#include "core/cpu_profiling.h"
#include "core/log.h"

namespace era_engine::physics
{
	static void clear_collider_from_collection(const BodyComponent* collider,
		physx::PxArray<SimulationEventCallback::CollidersPair>& collection)
	{
		const auto c = &collection[0];
		for (uint32_t i = 0; i < collection.size(); i++)
		{
			const SimulationEventCallback::CollidersPair& cc = c[i];
			if (cc.first == collider || cc.second == collider)
			{
				collection.remove(i--);
				if (collection.empty())
				{
					break;
				}
			}
		}
	}

	void* PhysicsAllocatorCallback::allocate(size_t size, const char* type_name, const char* filename, int line)
	{
		PX_ASSERT(size < GB(1));
		return _aligned_malloc(size, 16);
	}

	void PhysicsAllocatorCallback::deallocate(void* ptr)
	{
		_aligned_free(ptr);
	}

	physx::PxAgain OverlapCallback::processTouches(const physx::PxOverlapHit* buffer, physx::PxU32 nb_hits)
	{
		for (physx::PxU32 i = 0; i < nb_hits; ++i)
		{
			physx::PxRigidDynamic* rigid_dynamic = buffer[i].actor->is<physx::PxRigidDynamic>();
			if (rigid_dynamic != nullptr)
			{
				hit_actor = rigid_dynamic;
				break;
			}
		}
		return true;
	}

	physx::PxFilterFlags SimulationFilterCallback::pairFound(physx::PxU64 pairID,
		physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0, const physx::PxActor* a0, const physx::PxShape* s0,
		physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1, const physx::PxActor* a1, const physx::PxShape* s1,
		physx::PxPairFlags& pairFlags)
	{
		return physx::PxFilterFlags(physx::PxFilterFlag::eDEFAULT);
	};

	void SimulationFilterCallback::pairLost(physx::PxU64 pairID,
		physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
		physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
		bool objectRemoved)
	{
	};

	bool SimulationFilterCallback::statusChange(physx::PxU64& pairID, physx::PxPairFlags& pairFlags, physx::PxFilterFlags& filterFlags)
	{
		return false;
	};

	void ErrorReporter::reportError(physx::PxErrorCode::Enum code, const char* message, const char* file, int line)
	{
		if (message)
		{
			LOG_ERROR("PhysX Error> %s %s %u %s %s %s %u", message, "Code:", static_cast<int32>(code), "Source:", file, ":", line);

			std::ostringstream stream;
			stream << message;
			std::cout << stream.str() << "\n";
		}
		else
		{
			std::cerr << "PhysX Error! \n";
		}
	}

	void* ProfilerCallback::zoneStart(const char* eventName, bool detached, uint64_t contextId)
	{
#if ENABLE_CPU_PROFILING
		recordProfileEvent(profile_event_begin_block, eventName);
#endif
		return nullptr;
	}

	void ProfilerCallback::zoneEnd(void* profilerData, const char* eventName, bool detached, uint64_t contextId)
	{
#if ENABLE_CPU_PROFILING
		recordProfileEvent(profile_event_end_block, eventName);
#endif
	}
	
	physx::PxVec3 Collision::get_relative_velocity() const
	{
		return this_velocity - other_velocity;
	}

	void Collision::swap_objects()
	{
		if (!this_actor || !other_actor)
		{
			return;
		}
		std::swap(this_actor, other_actor);
		std::swap(this_velocity, other_velocity);
	}

	void SimulationEventCallback::clear()
	{
		new_collisions.clear();
		removed_collisions.clear();

		cct_collisions.clear();

		new_trigger_pairs.clear();
		lost_trigger_pairs.clear();

		broken_joints.clear();
	}

	void SimulationEventCallback::sendCollisionEvents()
	{
	}

	void SimulationEventCallback::sendTriggerEvents()
	{
	}

	void SimulationEventCallback::sendJointEvents()
	{
		for (auto* joint : broken_joints)
		{
			joint->state = JointComponent::State::BROKEN;
			if (joint->on_broken_callback)
			{
				joint->on_broken_callback(joint);
			}
		}
	}

	void SimulationEventCallback::onColliderRemoved(BodyComponent* collider)
	{
		clear_collider_from_collection(collider, new_trigger_pairs);
		clear_collider_from_collection(collider, lost_trigger_pairs);
	}

	void SimulationEventCallback::onJointRemoved(JointComponent* joint)
	{
		broken_joints.findAndReplaceWithLast(joint);
		broken_joints.popBack();
	}

	void SimulationEventCallback::onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count)
	{
		using namespace physx;
		for (uint32_t i = 0; i < count; i++)
		{
			PxJoint* joint = reinterpret_cast<PxJoint*>(constraints[i].externalReference);
			if (joint->userData)
			{
				broken_joints.pushBack(static_cast<JointComponent*>(joint->userData));
			}
		}
	}

	void SimulationEventCallback::onWake(physx::PxActor** actors, physx::PxU32 count)
	{
	}

	void SimulationEventCallback::onSleep(physx::PxActor** actors, physx::PxU32 count)
	{
	}

	void SimulationEventCallback::onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count)
	{

	}

	void SimulationEventCallback::onAdvance(const physx::PxRigidBody* const* body_buffer, const physx::PxTransform* pose_buffer, const physx::PxU32 count)
	{
	}

	void SimulationEventCallback::onContact(const physx::PxContactPairHeader& pair_header, const physx::PxContactPair* pairs, physx::PxU32 nb_pairs)
	{
		using namespace physx;
		const PxU32 buffer_size = PX_CONTACT_BUFFER_SIZE;
		PxContactPairPoint contacts[buffer_size];

		Collision collision{};
		PxContactPairExtraDataIterator iter(pair_header.extraDataStream, pair_header.extraDataStreamSize);

		for (PxU32 i = 0; i < nb_pairs; i++)
		{
			const PxContactPair& cp = pairs[i];
			PxU32 nb_contacts = pairs[i].extractContacts(contacts, buffer_size);

			const bool has_post_velocities = !cp.flags.isSet(PxContactPairFlag::eACTOR_PAIR_LOST_TOUCH);

			for (PxU32 j = 0; j < nb_contacts; j++)
			{
				const PxVec3& point = contacts[j].position;
				const PxVec3& impulse = contacts[j].impulse;
				PxU32 internal_face_index0 = contacts[j].internalFaceIndex0;
				PxU32 internal_face_index1 = contacts[j].internalFaceIndex1;

				collision.impulse += impulse;

				UNUSED(point);
				UNUSED(internal_face_index0);
				UNUSED(internal_face_index1);
			}

			collision.this_velocity = collision.other_velocity = PxVec3(0.0f);

			if (has_post_velocities && iter.nextItemSet())
			{
				if (iter.contactPairIndex != i)
				{
					continue;
				}
				if (iter.postSolverVelocity)
				{
					collision.this_velocity = iter.postSolverVelocity->linearVelocity[0];
					collision.other_velocity = iter.postSolverVelocity->linearVelocity[1];
				}
			}

			if (cp.events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
			{
				PxRigidActor* r1 = pair_header.actors[0]->is<PxRigidActor>();
				PxRigidActor* r2 = pair_header.actors[1]->is<PxRigidActor>();

				if (!r1 || !r2)
				{
					return;
				}

				PxRigidActor* actor1 = r1;
				PxRigidActor* actor2 = r2;

				BodyComponent* rb1 = PhysicsEngine::get_physics_core()->actors_map[actor1];
				BodyComponent* rb2 = PhysicsEngine::get_physics_core()->actors_map[actor2];

				if (!rb1 || !rb2)
				{
					return;
				}

				collision.this_actor = rb1;
				collision.other_actor = rb2;

				if (cp.flags & PxContactPairFlag::eACTOR_PAIR_HAS_FIRST_TOUCH)
				{
					new_collisions.pushBack(collision);
				}
			}
			else if (cp.events & physx::PxPairFlag::eNOTIFY_TOUCH_LOST)
			{
				PxRigidActor* r1 = pair_header.actors[0]->is<PxRigidActor>();
				PxRigidActor* r2 = pair_header.actors[1]->is<PxRigidActor>();

				if (!r1 || !r2)
				{
					return;
				}

				PxRigidActor* actor1 = r1;
				PxRigidActor* actor2 = r2;

				BodyComponent* rb1 = PhysicsEngine::get_physics_core()->actors_map[actor1];
				BodyComponent* rb2 = PhysicsEngine::get_physics_core()->actors_map[actor2];

				if (!rb1 || !rb2)
				{
					return;
				}

				collision.this_actor = rb1;
				collision.other_actor = rb2;

				if (cp.flags & PxContactPairFlag::eACTOR_PAIR_LOST_TOUCH)
				{
					removed_collisions.pushBack(collision);
				}
			}
		}
	}

	physx::PxAgain ExplodeOverlapCallback::processTouches(const physx::PxOverlapHit* buffer, physx::PxU32 nbHits)
	{
		using namespace physx;

		PxSceneWriteLock lock{ *PhysicsEngine::get_physics_core()->get_scene()};
		for (PxU32 i = 0; i < nbHits; ++i)
		{
			PxRigidActor* actor = buffer[i].actor;
			PxRigidDynamic* rigid_dynamic = actor->is<PxRigidDynamic>();
			if (rigid_dynamic && !(rigid_dynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
			{
				if (actor_buffer.find(rigid_dynamic) == actor_buffer.cend())
				{
					actor_buffer.insert(rigid_dynamic);
					const PxVec3 dr = rigid_dynamic->getGlobalPose().transform(rigid_dynamic->getCMassLocalPose()).p - world_position;
					const float distance = dr.magnitude();
					const float factor = PxClamp(1.0f - (distance * distance) / (radius * radius), 0.0f, 1.0f);
					const float impulse = factor * explosive_impulse * 1000.0f;
					const PxVec3 vel = dr.getNormalized() * impulse / rigid_dynamic->getMass();
					rigid_dynamic->setLinearVelocity(rigid_dynamic->getLinearVelocity() + vel);
				}
			}
		}
		return true;
	}

	CharacterControllerHitReportCallback::CharacterControllerHitReportCallback(World* world_)
		: world(world_)
	{
	}

	void CharacterControllerHitReportCallback::onShapeHit(const physx::PxControllerShapeHit& hit)
	{
		ASSERT(hit.controller != nullptr);
		ASSERT(hit.shape != nullptr);

		CharacterControllerComponent* cct_component = static_cast<CharacterControllerComponent*>(hit.controller->getUserData());
		ASSERT(cct_component != nullptr);

		if (!cct_component->handle_non_cct_contacts)
		{
			return;
		}

		Component* shape_component = static_cast<Component*>(hit.shape->userData);
		ASSERT(shape_component != nullptr);

		CharacterControllerCollision cct_collision_info;
		cct_collision_info.character_controller = cct_component;
		cct_collision_info.other_ptr = ComponentPtr{ shape_component };
		cct_collision_info.position = create_vec3(hit.worldPos);
		cct_collision_info.normal = create_vec3(hit.worldNormal);
		cct_collision_info.direction = create_vec3(hit.dir);
		cct_collision_info.length = hit.length;

		const ref<SimulationEventCallback>& event_callback = PhysicsEngine::get_physics_core()->simulation_event_callback;
		event_callback->cct_collisions.pushBack(cct_collision_info);
	}

	void CharacterControllerHitReportCallback::onControllerHit(const physx::PxControllersHit& hit)
	{
		ASSERT(hit.controller != nullptr);
		ASSERT(hit.other != nullptr);

		CharacterControllerComponent* this_cct_component = static_cast<CharacterControllerComponent*>(hit.controller->getUserData());
		ASSERT(this_cct_component != nullptr);

		Component* other_cct_component = static_cast<Component*>(hit.other->getUserData());
		ASSERT(other_cct_component != nullptr);

		CharacterControllerCollision cct_collision_info;
		cct_collision_info.character_controller = this_cct_component;
		cct_collision_info.other_ptr = ComponentPtr{ other_cct_component };
		cct_collision_info.position = create_vec3(hit.worldPos);
		cct_collision_info.normal = create_vec3(hit.worldNormal);
		cct_collision_info.direction = create_vec3(hit.dir);
		cct_collision_info.length = hit.length;

		const ref<SimulationEventCallback>& event_callback = PhysicsEngine::get_physics_core()->simulation_event_callback;
		event_callback->cct_collisions.pushBack(cct_collision_info);
	}

	void CharacterControllerHitReportCallback::onObstacleHit(const physx::PxControllerObstacleHit& hit)
	{
	}

}