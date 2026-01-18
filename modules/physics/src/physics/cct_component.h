#pragma once

#define PX_PHYSX_CHARACTER_STATIC_LIB

#include "physics_api.h"

#include "physics/body_component.h"
#include "physics/collision_types.h"

#include "ecs/component.h"

namespace era_engine::physics
{
    enum class CharacterControllerMoveMode : uint8
    {
        WALKING = 0,
        FLYING
    };

    enum class CharacterControllerCollisionFlags : uint8
    {
        NONE = 0,
        SIDES = 1 << 0,
        UP = 1 << 1,
        DOWN = 1 << 2
    };

    class ERA_PHYSICS_API CharacterControllerComponent final : public Component
    {
    public:
        enum class ClimbingMode : uint8
        {
            RADIUS_AND_STEP_OFFSET = 0,
            STEP_OFFSET
        };

        enum class NonWalkableMode : uint8
        {
            PREVENT_CLIMBING = 0,
            PREVENT_CLIMBING_AND_SLIDE
        };

        CharacterControllerComponent() = default;
        CharacterControllerComponent(ref<Entity::EcsData> _data);
        ~CharacterControllerComponent() override;

        ObservableMember<CollisionType> collision_type = CollisionType::NONE;
        ObservableMember<std::optional<uint32>> collision_filter_data = std::optional<uint32>{};

        float height = 1.2f;
        float radius = 0.3f;

        float step_offset = 0.5f;
        float contact_offset = 0.1f;

        float volume_growth = 1.5f;

        float slope_limit = deg2rad(30.0f);

        CharacterControllerMoveMode move_mode = CharacterControllerMoveMode::WALKING;
        ClimbingMode climbing_mode = ClimbingMode::STEP_OFFSET;
        NonWalkableMode non_walkable_mode = NonWalkableMode::PREVENT_CLIMBING;

        CharacterControllerCollisionFlags current_collision_flags = CharacterControllerCollisionFlags::NONE;

        bool handle_non_cct_contacts = false;

        ObservableMember<vec3> velocity = vec3::zero;
        ObservableMember<vec3> offset = vec3::zero;

        ERA_VIRTUAL_REFLECT(Component)

    private:
        physx::PxCapsuleController* controller = nullptr;

        friend class PhysicsSystem;
        friend class PhysicsUtils;
    };
}