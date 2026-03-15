#include "simple_motion_matching_module.h"

#include "engine/engine.h"
#include "ecs/world_system_scheduler.h"

#include <rttr/registration>

namespace era_engine
{

    SimpleMotionMatchingModule::SimpleMotionMatchingModule() noexcept
        : IModule("simple_motion_matching")
    {
    }

    SimpleMotionMatchingModule::~SimpleMotionMatchingModule()
    {
    }

    bool SimpleMotionMatchingModule::initialize(void* engine)
    {
        IModule::initialize(engine);

        ERA_MODULE_REGISTRATION

        return true;
    }

    bool SimpleMotionMatchingModule::terminate()
    {
        IModule::terminate();
        return true;
    }

    RTTR_PLUGIN_REGISTRATION
    {
        using namespace rttr;
        registration::class_<SimpleMotionMatchingModule>("SimpleMotionMatchingModule")
            .constructor<>()(policy::ctor::as_raw_ptr)
            .method("initialize", &SimpleMotionMatchingModule::initialize)
            .method("terminate", &SimpleMotionMatchingModule::terminate)
            (metadata("module_type", IModule::Type::ENGINE));
    }

}