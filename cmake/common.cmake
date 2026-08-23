set(CMAKE_CXX_STANDARD 20)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /arch:AVX2 /Zi /Gy /GF /EHsc")

set(ENGINE_DEFAULT_LIBS
d3d12.lib
uxtheme.lib
D3Dcompiler.lib
DXGI.lib
dxguid.lib
dxcompiler.lib
XAudio2.lib
Ws2_32.lib
)

function(assign_source_group)
    foreach(_source IN ITEMS ${ARGN})
        if (IS_ABSOLUTE "${_source}")
            file(RELATIVE_PATH _source_rel "${CMAKE_CURRENT_SOURCE_DIR}" "${_source}")
        else()
            set(_source_rel "${_source}")
        endif()
        get_filename_component(_source_path "${_source_rel}" PATH)
        string(REPLACE "/" "\\" _source_path_msvc "${_source_path}")
        source_group("${_source_path_msvc}" FILES "${_source}")
    endforeach()
endfunction(assign_source_group)

function(era_default_linkage name)
    target_compile_definitions(${name} PRIVATE _UNICODE
        UNICODE
        _CRT_SECURE_NO_WARNINGS
        ENABLE_CPU_PROFILING=0
        ENABLE_MESSAGE_LOG=1
        ENGINE_PATH=L"${ERA_ENGINE_PATH}"
        SHADER_BIN_DIR=L"${ERA_ENGINE_PATH}/modules/shaders/bin/Release/"
        USE_NETCORE
        MESH_SHADER_SUPPORTED
        ENABLE_DX_PROFILING=0
        ENABLE_FSR_WRAPPER
        ACL_USE_SJSON
        TRACY_ENABLE
        TRACY_ON_DEMAND
        TRACY_NO_CRASH_HANDLER
        TRACY_NO_CALLSTACK
        TRACY_NO_CODE_TRANSFER
        TRACY_NO_SAMPLING
        VISUALIZE_PHYSICS=1
    )

    target_link_libraries(${name} ${ENGINE_DEFAULT_LIBS})
    
    add_custom_command(TARGET ${name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${ERA_ENGINE_PATH}/resources/bin/$<CONFIGURATION>
        $<TARGET_FILE_DIR:${name}>
    )

    assign_source_group(${SOURCES_${name}} ${HEADERS_${name}})

    target_compile_options(${name} PRIVATE /wd4305 /wd4244 /wd4267 /wd4099 /wd4005 /wd4804 /wd4312)
endfunction()

function(declare_module name)
    message("-- Declare module ${name}.")

    file(GLOB_RECURSE SOURCES_${name} ${ERA_ENGINE_PATH}/modules/${name}/src/*.cpp)
    file(GLOB_RECURSE HEADERS_${name} ${ERA_ENGINE_PATH}/modules/${name}/src/*.h ${ERA_ENGINE_PATH}/modules/${name}/src/*.hpp)

    add_library(${name} SHARED ${SOURCES_${name}} ${HEADERS_${name}})
    target_include_directories(${name} PUBLIC ${ERA_ENGINE_PATH}/modules/${name}/src ${ERA_ENGINE_PATH}/tools/tracy/repo/public)

    era_default_linkage(${name})
endfunction()

function(declare_asset_module name)
    message("-- Declare asset module ${name}.")
endfunction()

function(declare_thirdparty_module name)
    message("-- Declare thirdparty module ${name}.")
endfunction()

function(declare_app name)
    message("-- Declare app ${name}.")

    file(GLOB_RECURSE SOURCES_${name} ${ERA_ENGINE_PATH}/apps/${name}/src/*.cpp)
    file(GLOB_RECURSE HEADERS_${name} ${ERA_ENGINE_PATH}/apps/${name}/src/*.h ${ERA_ENGINE_PATH}/modules/${name}/src/*.hpp)

    add_executable(${name} ${SOURCES_${name}} ${HEADERS_${name}})
    target_include_directories(${name} PUBLIC ${ERA_ENGINE_PATH}/apps/${name}/src ${ERA_ENGINE_PATH}/tools/tracy/repo/public)

    era_default_linkage(${name})
endfunction()

function(era_begin name target_type)
    message("-- Begin processing target ${name}...")

    if (${target_type} STREQUAL "APP")
        declare_app(${name})
        set_property(TARGET ${name} PROPERTY TARGET_TYPE ${target_type})
    elseif (${target_type} STREQUAL "MODULE")
        declare_module(${name})
        set_property(TARGET ${name} PROPERTY TARGET_TYPE ${target_type})
    elseif (${target_type} STREQUAL "THIRD-PARTY-MODULE")
        declare_thirdparty_module(${name})
    elseif (${target_type} STREQUAL "ASSET-MODULE")
        declare_asset_module(${name})
    else()
        message("Failed to create target with ${target_type} type!")
    endif()

    add_definitions(/FI"${ERA_ENGINE_PATH}/resources/common/era_common.h")
endfunction()

function(require_module target module)
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/${module}/src)

    message("Adding target ${module} to ${target} program...")

    # Link dependencies (if any)
    add_dependencies(${target} ${module})

    get_property(module_dependencies TARGET ${module} PROPERTY DEPENDENCIES)
    set_property(TARGET ${target} APPEND PROPERTY DEPENDENCIES ${module})
endfunction()

function(require_thirdparty_module target module)
    get_property(module_include_path TARGET ${module} PROPERTY INCLUDE_PATH)

    target_include_directories(${target} PUBLIC ${module_include_path})

    message("Adding target ${module} to ${target} program...")

    # Link dependencies (if any)
    add_dependencies(${target} ${module})

    get_property(module_dependencies TARGET ${module} PROPERTY DEPENDENCIES)
    set_property(TARGET ${target} APPEND PROPERTY DEPENDENCIES ${module})
endfunction()

function(era_end name)
    get_property(target_type TARGET ${name} PROPERTY TARGET_TYPE)
    message("-- End processing target ${name} with type: ${target_type}.")

    if(${target_type} STREQUAL "MODULE")
        set(INCLUDE_EXT TRUE)
    elseif(${target_type} STREQUAL "APP")
        set(INCLUDE_EXT TRUE)
    else()
        set(INCLUDE_EXT FALSE)
    endif()

    if(${INCLUDE_EXT})
        target_include_directories(${name} PUBLIC ${ERA_ENGINE_PATH}/modules/shaders)
        target_include_directories(${name} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-imgui)
        target_include_directories(${name} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-external)

        target_include_directories(${name} PUBLIC 
            ${ERA_ENGINE_PATH}/modules/shaders/common
            ${ERA_ENGINE_PATH}/modules/shaders/particle_systems
            ${ERA_ENGINE_PATH}/modules/shaders/rs)
    endif()

    get_property(target_dependencies TARGET ${name} PROPERTY DEPENDENCIES)
    list(REMOVE_DUPLICATES target_dependencies)
    target_link_libraries(${name} ${target_dependencies})
    message("${name} deps: ${target_dependencies}")

    foreach(dependency ${target_dependencies})
        get_property(dependency_type TARGET ${dependency} PROPERTY TARGET_TYPE)
        if(dependency_type STREQUAL "MODULE")
            set(DEP_BUILD_PATH ${ERA_ENGINE_PATH}/_build/modules/${dependency}/$<CONFIGURATION>/${dependency}.dll)
            add_custom_command(TARGET ${name} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${DEP_BUILD_PATH}
                $<TARGET_FILE_DIR:${name}>
            )
        elseif(dependency_type STREQUAL "THIRD-PARTY-MODULE")
            get_property(need_copy_bin TARGET ${dependency} PROPERTY NEED_COPY)

            if(need_copy_bin STREQUAL "true")
                get_property(bin_build_path TARGET ${dependency} PROPERTY BUILD_PATH)
                add_custom_command(TARGET ${name} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_directory
                    ${bin_build_path}
                    $<TARGET_FILE_DIR:${name}>
                )
            endif()
        endif()
    endforeach()
endfunction()

set(PHYSX_DEBUG_LIBS
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/LowLevel_static_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/LowLevelAABB_static_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/LowLevelDynamics_static_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/PhysX_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/PhysXCharacterKinematic_static_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/PhysXPvdSDK_static_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/PhysXVehicle2_static_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/PhysXCooking_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/PhysXCommon_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/PhysXExtensions_static_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/PhysXFoundation_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/PhysXTask_static_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/PhysXVehicle_static_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/PVDRuntime_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/SceneQuery_static_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/SimulationController_static_64.lib
debug ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Debug/SnippetUtils_static_64.lib
)

set(PHYSX_RELEASE_LIBS
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/LowLevel_static_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/LowLevelAABB_static_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/LowLevelDynamics_static_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/PhysX_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/PhysXCharacterKinematic_static_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/PhysXPvdSDK_static_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/PhysXVehicle2_static_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/PhysXCooking_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/PhysXCommon_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/PhysXExtensions_static_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/PhysXFoundation_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/PhysXTask_static_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/PhysXVehicle_static_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/PVDRuntime_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/SceneQuery_static_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/SimulationController_static_64.lib
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-physx/lib/Release/SnippetUtils_static_64.lib
)

set(PHYSX_BLAST_RELEASE_LIBS
${ERA_ENGINE_PATH}/modules/thirdparty-physx/blast/bin/NvBlast.lib
${ERA_ENGINE_PATH}/modules/thirdparty-physx/blast/bin/NvBlastExtAssetUtils.lib
${ERA_ENGINE_PATH}/modules/thirdparty-physx/blast/bin/NvBlastExtAuthoring.lib
${ERA_ENGINE_PATH}/modules/thirdparty-physx/blast/bin/NvBlastExtSerialization.lib
${ERA_ENGINE_PATH}/modules/thirdparty-physx/blast/bin/NvBlastExtShaders.lib
${ERA_ENGINE_PATH}/modules/thirdparty-physx/blast/bin/NvBlastExtStress.lib
${ERA_ENGINE_PATH}/modules/thirdparty-physx/blast/bin/NvBlastExtTkSerialization.lib
${ERA_ENGINE_PATH}/modules/thirdparty-physx/blast/bin/NvBlastGlobals.lib
${ERA_ENGINE_PATH}/modules/thirdparty-physx/blast/bin/NvBlastTk.lib
)

function(require_physx target)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${ERA_ENGINE_PATH}/modules/thirdparty-physx/bin/$<CONFIGURATION>
        $<TARGET_FILE_DIR:${target}>
    )
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${ERA_ENGINE_PATH}/modules/thirdparty-physx/blast/bin
        $<TARGET_FILE_DIR:${target}>
    )
    target_link_libraries(${target} ${PHYSX_RELEASE_LIBS} ${PHYSX_DEBUG_LIBS} ${PHYSX_BLAST_RELEASE_LIBS})
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-physx/src/physx)
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-physx/src/blast/include)
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-physx/src/blast/include/shared/NvFoundation)
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-physx/src/blast/include/lowlevel)
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-physx/src/blast/include/globals)
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-physx/src/blast/include/toolkit)
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-physx/src/blast/include/extensions)
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-physx/src/blast/include/extensions/assetutils)
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-physx/src/blast/include/extensions/serialization)
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-physx/src/blast/include/extensions/stress)
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-physx/src/blast/include/extensions/shaders)
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-physx/src/blast/include/extensions/authoring)
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-physx/src/blast/include/extensions/authoringCommon)
endfunction()

set(ASSIMP_DEBUG_LIBS
debug ${ERA_ENGINE_PATH}/modules/thirdparty-assimp/lib/Debug/assimp-vc143-mtd.lib
)

set(ASSIMP_RELEASE_LIBS
optimized ${ERA_ENGINE_PATH}/modules/thirdparty-assimp/lib/Release/assimp-vc143-mt.lib
)

function(require_assimp target)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${ERA_ENGINE_PATH}/modules/thirdparty-assimp/bin/$<CONFIGURATION>
        $<TARGET_FILE_DIR:${target}>
    )

    target_link_libraries(${target} ${ASSIMP_RELEASE_LIBS} ${ASSIMP_DEBUG_LIBS})
    target_include_directories(${target} PUBLIC ${ERA_ENGINE_PATH}/modules/thirdparty-assimp/include)
endfunction()