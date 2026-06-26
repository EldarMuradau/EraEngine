#include "physics/physical_animation/dismemberment/ragdoll_dismemberment_component.h"

#include <rttr/registration>

namespace era_engine::physics
{
	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<RagdollDismembermentComponent>("RagdollDismembermentComponent")
			.constructor<>();

		registration::class_<RagdollDismembermentLimbComponent>("RagdollDismembermentLimbComponent")
			.constructor<>();
	}

	RagdollDismembermentLimbComponent::RagdollDismembermentLimbComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
	}

	void RagdollDismembermentLimbComponent::apply_damage(float damage)
	{
		if (!can_be_dismembered)
		{
			return;
		}

		if (damage <= 0.0f)
		{
			return;
		}

		if (is_dismembered())
		{
			return;
		}

		current_health -= damage;

		if (current_health <= 0.0f)
		{
			trigger_dismembering();
			current_health = 0.0f;
		}
	}

	void RagdollDismembermentLimbComponent::trigger_dismembering()
	{
		if (!can_be_dismembered || is_dismembered())
		{
			return;
		}
		state = DismemberState::PENDING;
	}

	bool RagdollDismembermentLimbComponent::is_dismembered() const
	{
		return state == DismemberState::DISMEMBERED;
	}

	bool RagdollDismembermentLimbComponent::is_pending() const
	{
		return state == DismemberState::PENDING;
	}

	DismemberState RagdollDismembermentLimbComponent::get_current_state() const
	{
		return state;
	}

	RagdollDismembermentComponent::RagdollDismembermentComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
	}
}