#include "dynx_physics_module.h"

#include "engine/engine.h"

#include "ecs/world_system_scheduler.h"

#include <rttr/registration>

namespace era_engine
{

    DynXPhysicsModule::DynXPhysicsModule() noexcept
        : IModule("physics")
    {
    }

    DynXPhysicsModule::~DynXPhysicsModule()
    {
    }

    bool DynXPhysicsModule::initialize(void* engine)
    {
        IModule::initialize(engine);

        ERA_MODULE_REGISTRATION

        return true;
    }

    bool DynXPhysicsModule::terminate()
    {
        IModule::terminate();
        return true;
    }

    RTTR_PLUGIN_REGISTRATION
    {
        using namespace rttr;
        registration::class_<DynXPhysicsModule>("DynXPhysicsModule")
            .constructor<>()(policy::ctor::as_raw_ptr)
            .method("initialize", &DynXPhysicsModule::initialize)
            .method("terminate", &DynXPhysicsModule::terminate)
            (metadata("module_type", IModule::Type::ENGINE));
    }

}