#include "physics/material.h"
#include "physics/core/physics.h"

namespace era_engine::physics
{
	PhysicsMaterial::PhysicsMaterial(physx::PxPhysics* physics, float _restitution, float _static_friction, float _dynamic_friction)
	{
		material = physics->createMaterial(_static_friction, _dynamic_friction, _restitution);
	}

	PhysicsMaterial::~PhysicsMaterial()
	{
		release();
	}

	void PhysicsMaterial::release()
	{
		if(material != nullptr && material->isReleasable())
		{
			PX_RELEASE(material)
		}
	}

	physx::PxMaterial* PhysicsMaterial::get_native_material() const
	{
		return material;
	}
}