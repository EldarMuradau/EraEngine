// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "physics/core/physics.h"
#include "physics/body_component.h"
#include "physics/shape_component.h"
#include "physics/cct_component.h"
#include "physics/core/physics_utils.h"
#include "physics/shape_utils.h"

#include "ecs/world.h"
#include "ecs/base_components/transform_component.h"

#include <rttr/registration>

namespace era_engine::physics
{

	RTTR_REGISTRATION
	{
		using namespace rttr;
		registration::class_<BodyComponent>("BodyComponent")
			.constructor<>();

		registration::class_<DynamicBodyComponent>("DynamicBodyComponent")
			.constructor<>();

		registration::class_<StaticBodyComponent>("StaticBodyComponent")
			.constructor<>();
	}

	BodyComponent::BodyComponent(ref<Entity::EcsData> _data)
		: Component(_data)
	{
	}

	BodyComponent::~BodyComponent()
	{
	}

	physx::PxRigidActor* BodyComponent::get_rigid_actor() const
	{
		return actor;
	}

	void BodyComponent::detach_shape(physx::PxShape* shape)
	{
		if (actor != nullptr)
		{
			PhysicsEngine::execute_write([&]() {
				actor->detachShape(*shape);
				});
		}
	}

	DynamicBodyComponent::DynamicBodyComponent(ref<Entity::EcsData> _data)
		: BodyComponent(_data)
	{
		ComponentPtr this_component = ComponentPtr(this);

		simulated.set_component(this_component);
		kinematic.set_component(this_component);
		ccd.set_component(this_component);
		use_gravity.set_component(this_component);
		constraints.set_component(this_component);
		linear_damping.set_component(this_component);
		angular_damping.set_component(this_component);
		linear_velocity.set_component(this_component);
		angular_velocity.set_component(this_component);
		center_of_mass.set_component(this_component);
		mass.set_component(this_component);
		max_angular_velocity.set_component(this_component);
		max_linear_velocity.set_component(this_component);
		max_contact_impulse.set_component(this_component);
		max_depenetration_velocity.set_component(this_component);
		sleep_threshold.set_component(this_component);
		stabilization_threshold.set_component(this_component);
		solver_velocity_iterations_count.set_component(this_component);
		solver_position_iterations_count.set_component(this_component);
		mass_space_inertia_tensor.set_component(this_component);
		kinematic_motion_type.set_component(this_component);
	}

	DynamicBodyComponent::~DynamicBodyComponent()
	{
	}

	physx::PxRigidDynamic* DynamicBodyComponent::get_rigid_dynamic() const
	{
		return actor == nullptr ? nullptr : actor->is<physx::PxRigidDynamic>();
	}

	vec3 DynamicBodyComponent::get_physics_position() const
	{
		using namespace physx;

		PxVec3 pos;

		PhysicsEngine::execute_read([&]() {
			pos = actor->getGlobalPose().p;
			});

		return create_vec3(pos);
	}

	void DynamicBodyComponent::wake_up()
	{
		using namespace physx;

		if (actor == nullptr)
		{
			return;
		}

		if (PxRigidDynamic* body = actor->is<PxRigidDynamic>())
		{
			if(simulated)
			{
				PhysicsEngine::execute_write([&]() {
					body->wakeUp();
					});
			}
		}
	}

	StaticBodyComponent::StaticBodyComponent(ref<Entity::EcsData> _data)
		: BodyComponent(_data)
	{
		simulated.set_component(ComponentPtr(this));
	}

	StaticBodyComponent::~StaticBodyComponent()
	{
	}

	physx::PxRigidStatic* StaticBodyComponent::get_rigid_static() const
	{
		return actor->is<physx::PxRigidStatic>();
	}

}