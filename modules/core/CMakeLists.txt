include(${CMAKE_SOURCE_DIR}/cmake/common.cmake)

era_begin(core "MODULE")
    require_module(core imgui)
    require_module(core base)
    require_thirdparty_module(core EnTT)
    require_thirdparty_module(core yaml-cpp)
    require_thirdparty_module(core rttr_core)
    require_thirdparty_module(core DirectXTex)

    require_assimp(core)

    target_compile_definitions(core PRIVATE ERA_CORE TRACY_IMPORTS)
era_end(core)