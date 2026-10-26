#include "physics/material.h"
#include "physics/core/physics.h"

namespace era_engine::physics
{
    static_assert(static_cast<int>(PhysicsMaterialCombineMode::AVERAGE) == physx::PxCombineMode::eAVERAGE &&
                  static_cast<int>(PhysicsMaterialCombineMode::MIN) == physx::PxCombineMode::eMIN &&
                  static_cast<int>(PhysicsMaterialCombineMode::MULTIPLY) == physx::PxCombineMode::eMULTIPLY &&
                  static_cast<int>(PhysicsMaterialCombineMode::MAX) == physx::PxCombineMode::eMAX);

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
        if (material != nullptr && material->isReleasable())
        {
            PX_RELEASE(material)
        }
    }

    void PhysicsMaterial::set_friction_combine_mode(PhysicsMaterialCombineMode mode)
    {
        material->setFrictionCombineMode(static_cast<physx::PxCombineMode::Enum>(mode));
    }

    void PhysicsMaterial::set_restitution_combine_mode(PhysicsMaterialCombineMode mode)
    {
        material->setRestitutionCombineMode(static_cast<physx::PxCombineMode::Enum>(mode));
    }

    physx::PxMaterial* PhysicsMaterial::get_native_material() const
    {
        return material;
    }
}