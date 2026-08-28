cmake_minimum_required(VERSION 3.12)

set(ERA_ENGINE_PATH ${CMAKE_SOURCE_DIR})

include(cmake/common.cmake)
include(cmake/deploy.cmake)

project(EraEngine VERSION 1.0)

enable_testing()

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

add_subdirectory(modules/thirdparty-entt)
add_subdirectory(modules/thirdparty-yaml-cpp)
add_subdirectory(modules/thirdparty-directxtex)
add_subdirectory(modules/thirdparty-rttr)
add_subdirectory(modules/thirdparty-gtest)

add_subdirectory(modules/imgui)
add_subdirectory(modules/base)
add_subdirectory(modules/core)
add_subdirectory(modules/physics)
add_subdirectory(modules/dynx)
add_subdirectory(modules/simple_motion_matching)

add_subdirectory(apps/editor)
add_subdirectory(apps/example_game)
add_subdirectory(apps/unittests)

era_add_deploy_target(editor)
era_add_deploy_target(example_game)
era_add_deploy_target(unittests)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)