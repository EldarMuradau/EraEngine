#include "physics_module.h"

#include "physics/core/physics.h"
#include "physics/physx_api.h"

#include "engine/engine.h"

#include "ecs/world_system_scheduler.h"

#include <rttr/registration>

namespace era_engine
{

    PhysicsModule::PhysicsModule() noexcept
        : IModule("physics")
    {
    }

    PhysicsModule::~PhysicsModule()
    {
    }

    bool PhysicsModule::initialize(void* engine)
    {
        using namespace physics;

        IModule::initialize(engine);

        PhysicsDescriptor desc;
        //desc.broad_phase = physx::PxBroadPhaseType::eGPU;
        //desc.enable_tgs_solver = false;
        
#if _DEBUG
        desc.enable_pvd = true;
#else
        desc.enable_pvd = false;
#endif

        ref<Physics> physics_core = make_ref<Physics>(desc);
        physics_core->init_scene();
        physics_core->start();

        PhysicsEngine::physics_core = physics_core;

        ERA_MODULE_REGISTRATION

        return true;
    }

    bool PhysicsModule::terminate()
    {
        physics::PhysicsEngine::get_physics_core()->release_locked();

        IModule::terminate();
        return true;
    }

    RTTR_PLUGIN_REGISTRATION
    {
        using namespace rttr;
        registration::class_<PhysicsModule>("PhysicsModule")
            .constructor<>()(policy::ctor::as_raw_ptr)
            .method("initialize", &PhysicsModule::initialize)
            .method("terminate", &PhysicsModule::terminate)
            (metadata("module_type", IModule::Type::ENGINE));
    }

}