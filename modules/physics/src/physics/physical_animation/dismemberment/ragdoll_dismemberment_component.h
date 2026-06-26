#pragma once

#include "physics_api.h"
#include "physics/physical_animation/dismemberment/ragdoll_dismemberment_profile.h"

#include <core/math.h>

#include <ecs/component.h>
#include <ecs/observable_member.h>

namespace era_engine::physics
{
	enum class DismemberState : uint8
	{
		NONE = 0,
		PENDING,
		PARTIAL_DISMEMBERED,
		DISMEMBERED,
	};

	class ERA_PHYSICS_API RagdollDismembermentLimbComponent : public Component
	{
	public:
		RagdollDismembermentLimbComponent() = default;
		RagdollDismembermentLimbComponent(ref<Entity::EcsData> _data);

		void apply_damage(float damage);

		void trigger_dismembering();

		bool is_dismembered() const;
		bool is_pending() const;

		DismemberState get_current_state() const;

	public:
		EntityPtr ragdoll_ptr;

		float current_health = 0.0f;

        ERA_VIRTUAL_REFLECT(Component)

	private:
		DismemberState state = DismemberState::NONE;

		bool can_be_dismembered = true;

		friend class RagdollDismembermentSystem;
	};

	class ERA_PHYSICS_API RagdollDismembermentComponent : public Component
	{
	public:
		RagdollDismembermentComponent() = default;
		RagdollDismembermentComponent(ref<Entity::EcsData> _data);

	public:
		ref<RagdollDismembermentProfile> dismemberment_profile;

        ERA_VIRTUAL_REFLECT(Component)

	private:
		bool loaded = false;

		friend class RagdollDismembermentSystem;
	};
}