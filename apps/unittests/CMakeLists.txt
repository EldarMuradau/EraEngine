include(${CMAKE_SOURCE_DIR}/cmake/common.cmake)

enable_testing()

era_begin(unittests "APP")
    require_thirdparty_module(unittests EnTT)
    require_thirdparty_module(unittests yaml-cpp)
    require_thirdparty_module(unittests rttr_core)
    require_thirdparty_module(unittests DirectXTex)
    require_module(unittests imgui)
    require_module(unittests base)
    require_module(unittests core)
    require_module(unittests physics)
    require_module(unittests dynx)

    require_physx(unittests)

    target_include_directories(unittests PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-gtest/googletest/include)

    find_library(gtest NAMES gtest PATHS ${ERA_ENGINE_PATH}/modules/thirdparty-gtest/googletest)
    find_library(gtest_main NAMES gtest_main PATHS ${ERA_ENGINE_PATH}/modules/thirdparty-gtest/googletest)
    
    target_link_libraries(unittests gtest gtest_main)

    add_dependencies(unittests gtest)
    add_dependencies(unittests gtest_main)

    add_test(NAME EngineUnittests COMMAND unittests)

era_end(unittests)