# -----------------------------------------------------------------------------
# NOTE ABOUT METADATA REGISTRATION / INHERITANCE
#
# This file has been extended to support a configuration flow where the final
# GVSOC model target name is not always known inside the module-local
# CMakeLists.txt.
#
# In the original flow, helper functions such as:
#   - vp_model_include_directories()
#   - vp_model_link_libraries()
#   - vp_model_link_options()
#   - vp_model_compile_options()
#   - vp_model_compile_definitions()
#
# directly applied properties to an already-known model name.
#
# However, in the config-driven GVSOC build flow, the actual model target is
# often instantiated later by the top-level CMake logic through:
#
#   vp_model(NAME ${component} SOURCES "${SRCS_LIST}")
#
# where `${component}` is derived from the generated GVSOC configuration
# (e.g. gapy/gvrun output), and may differ from the logical name used inside
# the module-local CMakeLists.txt.
#
# This creates a practical issue for out-of-tree or custom modules: the module
# knows its own sources and external dependencies, but it does not necessarily
# know the final target name selected by the top-level configuration flow.
#
# To solve this cleanly without patching the root CMakeLists, this file now
# supports a two-step mechanism:
#
#   1) Metadata registration
#      Module-local helper calls always register build metadata
#      (sources, include directories, link libraries, link options, compile
#      options, compile definitions) in a global CMake registry, even if the
#      final target is not created yet.
#
#   2) Metadata inheritance
#      When vp_model() is later called by the top-level flow to instantiate the
#      real model target, the implementation compares the model sources against
#      the previously registered module sources. If a match is found, the
#      registered metadata is automatically applied to the real target.
#
# The matching is currently performed by source file basename. This keeps the
# mechanism simple and minimally invasive, while being sufficient for modules
# whose source filenames are unique within the build.
#
# The main goal of this extension is to let each module describe its own build
# requirements locally, while preserving the existing top-level GVSOC
# configuration flow unchanged.
# -----------------------------------------------------------------------------

if(POLICY CMP0177)
    cmake_policy(SET CMP0177 NEW)
endif()

set(VP_TARGET_TYPES "" CACHE INTERNAL "contains the types of target")

# -----------------------------------------------------------------------------
# Internal registry helpers
# -----------------------------------------------------------------------------

function(_vp_append_global_property prop)
    foreach(item IN LISTS ARGN)
        if(NOT "${item}" STREQUAL "")
            set_property(GLOBAL APPEND PROPERTY ${prop} "${item}")
        endif()
    endforeach()
endfunction()

function(_vp_set_model_property name suffix)
    set(prop "VP_MODEL_PROP_${suffix}_${name}")
    set_property(GLOBAL PROPERTY ${prop} "${ARGN}")
endfunction()

function(_vp_get_model_property out name suffix)
    set(prop "VP_MODEL_PROP_${suffix}_${name}")
    get_property(val GLOBAL PROPERTY ${prop})
    if(NOT val)
        set(val "")
    endif()
    set(${out} "${val}" PARENT_SCOPE)
endfunction()

function(_vp_register_model_metadata name)
    cmake_parse_arguments(
        VP_META
        ""
        ""
        "SOURCES;INCLUDE_DIRS;LINK_LIBS;LINK_OPTIONS;COMPILE_OPTIONS;COMPILE_DEFINITIONS"
        ${ARGN}
    )

    get_property(_registered GLOBAL PROPERTY VP_MODEL_REGISTERED_NAMES)
    if(NOT _registered)
        set(_registered "")
    endif()

    list(FIND _registered "${name}" _idx)
    if(_idx EQUAL -1)
        set_property(GLOBAL APPEND PROPERTY VP_MODEL_REGISTERED_NAMES "${name}")
    endif()

    if(VP_META_SOURCES)
        _vp_set_model_property("${name}" "SOURCES" "${VP_META_SOURCES}")
    endif()

    if(VP_META_INCLUDE_DIRS)
        _vp_append_global_property("VP_MODEL_PROP_INCLUDE_DIRS_${name}" ${VP_META_INCLUDE_DIRS})
    endif()

    if(VP_META_LINK_LIBS)
        _vp_append_global_property("VP_MODEL_PROP_LINK_LIBS_${name}" ${VP_META_LINK_LIBS})
    endif()

    if(VP_META_LINK_OPTIONS)
        _vp_append_global_property("VP_MODEL_PROP_LINK_OPTIONS_${name}" ${VP_META_LINK_OPTIONS})
    endif()

    if(VP_META_COMPILE_OPTIONS)
        _vp_append_global_property("VP_MODEL_PROP_COMPILE_OPTIONS_${name}" ${VP_META_COMPILE_OPTIONS})
    endif()

    if(VP_META_COMPILE_DEFINITIONS)
        _vp_append_global_property("VP_MODEL_PROP_COMPILE_DEFINITIONS_${name}" ${VP_META_COMPILE_DEFINITIONS})
    endif()
endfunction()

function(_vp_lists_match_by_basename out list_a list_b)
    set(_match FALSE)

    foreach(_a IN LISTS list_a)
        get_filename_component(_a_name "${_a}" NAME)
        foreach(_b IN LISTS list_b)
            get_filename_component(_b_name "${_b}" NAME)
            if(_a_name STREQUAL _b_name)
                set(_match TRUE)
            endif()
        endforeach()
    endforeach()

    set(${out} ${_match} PARENT_SCOPE)
endfunction()

function(_vp_inherit_registered_metadata model_name)
    cmake_parse_arguments(
        VP_INHERIT
        ""
        ""
        "SOURCES"
        ${ARGN}
    )

    get_property(_registered GLOBAL PROPERTY VP_MODEL_REGISTERED_NAMES)
    if(NOT _registered)
        return()
    endif()

    foreach(_registered_name IN LISTS _registered)
        # Do not try to inherit from self
        if(_registered_name STREQUAL "${model_name}")
            continue()
        endif()

        _vp_get_model_property(_registered_sources "${_registered_name}" "SOURCES")

        if(NOT _registered_sources)
            continue()
        endif()

        _vp_lists_match_by_basename(_matched "${VP_INHERIT_SOURCES}" "${_registered_sources}")

        if(NOT _matched)
            continue()
        endif()

        get_property(_include_dirs GLOBAL PROPERTY "VP_MODEL_PROP_INCLUDE_DIRS_${_registered_name}")
        get_property(_link_libs    GLOBAL PROPERTY "VP_MODEL_PROP_LINK_LIBS_${_registered_name}")
        get_property(_link_opts    GLOBAL PROPERTY "VP_MODEL_PROP_LINK_OPTIONS_${_registered_name}")
        get_property(_copts        GLOBAL PROPERTY "VP_MODEL_PROP_COMPILE_OPTIONS_${_registered_name}")
        get_property(_cdefs        GLOBAL PROPERTY "VP_MODEL_PROP_COMPILE_DEFINITIONS_${_registered_name}")

        if(_include_dirs)
            vp_model_include_directories(NAME ${model_name} DIRECTORY ${_include_dirs} FORCE_BUILD 1)
        endif()

        if(_link_opts)
            vp_model_link_options(NAME ${model_name} OPTIONS ${_link_opts} FORCE_BUILD 1)
        endif()

        if(_link_libs)
            foreach(_lib IN LISTS _link_libs)
                vp_model_link_libraries(NAME ${model_name} LIBRARY ${_lib} FORCE_BUILD 1)
            endforeach()
        endif()

        if(_copts)
            vp_model_compile_options(NAME ${model_name} OPTIONS ${_copts} FORCE_BUILD 1)
        endif()

        if(_cdefs)
            vp_model_compile_definitions(NAME ${model_name} DEFINITIONS ${_cdefs} FORCE_BUILD 1)
        endif()
    endforeach()
endfunction()

# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------

function(vp_set_target_types)
    cmake_parse_arguments(
        VP_TARGET_TYPES
        ""
        "BUILD_OPTIMIZED;BUILD_PROFILE;BUILD_DEBUG"
        ""
        ${ARGN}
        )
    if(${BUILD_OPTIMIZED} AND NOT "_optim" IN_LIST VP_TARGET_TYPES)
        set(VP_TARGET_TYPES ${VP_TARGET_TYPES} "_optim" CACHE INTERNAL "")
    endif()
    if(${BUILD_PROFILE} AND NOT "_profile" IN_LIST VP_TARGET_TYPES)
        set(VP_TARGET_TYPES ${VP_TARGET_TYPES} "_profile" CACHE INTERNAL "")
    endif()
    if(${BUILD_DEBUG} AND NOT "_debug" IN_LIST VP_TARGET_TYPES)
        set(VP_TARGET_TYPES ${VP_TARGET_TYPES} "_debug" CACHE INTERNAL "")
    endif()
    if(${BUILD_OPTIMIZED_M32} AND NOT "_optim_m32" IN_LIST VP_TARGET_TYPES)
        set(VP_TARGET_TYPES ${VP_TARGET_TYPES} "_optim_m32" CACHE INTERNAL "")
    endif()
    if(${BUILD_DEBUG_M32} AND NOT "_debug_m32" IN_LIST VP_TARGET_TYPES)
        set(VP_TARGET_TYPES ${VP_TARGET_TYPES} "_debug_m32" CACHE INTERNAL "")
    endif()
endfunction()

# vp_block function
function(vp_block)
    cmake_parse_arguments(
        VP_MODEL
        ""
        "NAME;OUTPUT_NAME"
        "SOURCES;INCLUDES"
        ${ARGN}
        )
    #message(STATUS "vp_model: name=\"${VP_MODEL_NAME}\", output_name=\"${VP_MODEL_OUTPUT_NAME}\" prefix=\"${VP_MODEL_PREFIX}\", srcs=\"${VP_MODEL_SOURCES}\", incs=\"${VP_MODEL_INCLUDES}\"")

    # TODO verify arguments
    set(VP_MODEL_NAME_OPTIM "${VP_MODEL_NAME}_optim")
    set(VP_MODEL_NAME_PROFILE "${VP_MODEL_NAME}_profile")
    set(VP_MODEL_NAME_DEBUG "${VP_MODEL_NAME}_debug")
    set(VP_MODEL_NAME_OPTIM_M32 "${VP_MODEL_NAME}_optim_m32")
    set(VP_MODEL_NAME_DEBUG_M32 "${VP_MODEL_NAME}_debug_m32")

    # ==================
    # Optimized models
    # ==================
    if(${BUILD_OPTIMIZED})
        add_library(${VP_MODEL_NAME_OPTIM} STATIC ${VP_MODEL_SOURCES})
        target_link_libraries(${VP_MODEL_NAME_OPTIM} PRIVATE gvsoc)
        set_target_properties(${VP_MODEL_NAME_OPTIM} PROPERTIES PREFIX "")
        target_compile_options(${VP_MODEL_NAME_OPTIM} PRIVATE -fno-stack-protector -D__GVSOC__)

        target_include_directories(${VP_MODEL_NAME_OPTIM} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

        foreach(X IN LISTS GVSOC_MODULES)
            target_include_directories(${VP_MODEL_NAME_OPTIM} PRIVATE ${X})
        endforeach()

        foreach(subdir ${VP_MODEL_INCLUDE_DIRS})
            target_include_directories(${VP_MODEL_NAME_OPTIM} PRIVATE ${subdir})
        endforeach()

        if(VP_MODEL_OUTPUT_NAME)
            set(RENAME_OPTIM_NAME ${VP_MODEL_OUTPUT_NAME})
        else()
            set(RENAME_OPTIM_NAME ${VP_MODEL_NAME_OPTIM})
        endif()

        install(
            FILES $<TARGET_FILE:${VP_MODEL_NAME_OPTIM}>
            DESTINATION  "${GVSOC_MODELS_INSTALL_FOLDER}/${GVSOC_MODELS_OPTIM_INSTALL_FOLDER}/${VP_MODEL_PREFIX}"
            RENAME "${RENAME_OPTIM_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}"
            )
    endif()

    if(${BUILD_OPTIMIZED_M32})
        add_library(${VP_MODEL_NAME_OPTIM_M32} STATIC ${VP_MODEL_SOURCES})
        target_link_libraries(${VP_MODEL_NAME_OPTIM_M32} PRIVATE gvsoc_m32)
        set_target_properties(${VP_MODEL_NAME_OPTIM_M32} PROPERTIES PREFIX "")
        target_compile_options(${VP_MODEL_NAME_OPTIM_M32} PRIVATE "-D__GVSOC__")
        target_compile_options(${VP_MODEL_NAME_OPTIM_M32} PRIVATE -m32 -D__M32_MODE__=1)
        target_link_options(${VP_MODEL_NAME_OPTIM_M32} PRIVATE -m32)
        target_include_directories(${VP_MODEL_NAME_OPTIM_M32} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

        foreach(X IN LISTS GVSOC_MODULES)
            target_include_directories(${VP_MODEL_NAME_OPTIM_M32} PRIVATE ${X})
        endforeach()

        foreach(subdir ${VP_MODEL_INCLUDE_DIRS})
            target_include_directories(${VP_MODEL_NAME_OPTIM_M32} PRIVATE ${subdir})
        endforeach()

        if(VP_MODEL_OUTPUT_NAME)
            set(RENAME_OPTIM_M32_NAME ${VP_MODEL_OUTPUT_NAME})
        else()
            set(RENAME_OPTIM_M32_NAME ${VP_MODEL_NAME_OPTIM_M32})
        endif()

        install(
            FILES $<TARGET_FILE:${VP_MODEL_NAME_OPTIM_M32}>
            DESTINATION  "${GVSOC_MODELS_INSTALL_FOLDER}/${GVSOC_MODELS_OPTIM_M32_INSTALL_FOLDER}/${VP_MODEL_PREFIX}"
            RENAME "${RENAME_OPTIM_M32_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}"
            )
    endif()

    # ==================
    # Profile models
    # ==================
    if(${BUILD_PROFILE})
        add_library(${VP_MODEL_NAME_PROFILE} STATIC ${VP_MODEL_SOURCES})
        target_link_libraries(${VP_MODEL_NAME_PROFILE} PRIVATE gvsoc_profile)
        set_target_properties(${VP_MODEL_NAME_PROFILE} PROPERTIES PREFIX "")
        target_compile_options(${VP_MODEL_NAME_PROFILE} PRIVATE "-D__GVSOC__")
        # TODO VP_TRACE_ACTIVE should be removed as soon as traces have been switch to events
        target_compile_definitions(${VP_MODEL_NAME_PROFILE} PRIVATE -DGVSOC_CONFIG_EVENT_ACTIVE=1 -DVP_TRACE_ACTIVE=1)

        target_include_directories(${VP_MODEL_NAME_PROFILE} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

        foreach(X IN LISTS GVSOC_MODULES)
            target_include_directories(${VP_MODEL_NAME_PROFILE} PRIVATE ${X})
        endforeach()

        foreach(subdir ${VP_MODEL_INCLUDE_DIRS})
            target_include_directories(${VP_MODEL_NAME_PROFILE} PRIVATE ${subdir})
        endforeach()

        if(VP_MODEL_OUTPUT_NAME)
            set(RENAME_PROFILE_NAME ${VP_MODEL_OUTPUT_NAME})
        else()
            set(RENAME_PROFILE_NAME ${VP_MODEL_NAME_PROFILE})
        endif()

        install(
            FILES $<TARGET_FILE:${VP_MODEL_NAME_PROFILE}>
            DESTINATION  "${GVSOC_MODELS_INSTALL_FOLDER}/${GVSOC_MODELS_PROFILE_INSTALL_FOLDER}/${VP_MODEL_PREFIX}"
            RENAME "${RENAME_PROFILE_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}"
            )
    endif()

    # ==================
    # Debug models
    # ==================
    if(${BUILD_DEBUG})
        add_library(${VP_MODEL_NAME_DEBUG} STATIC ${VP_MODEL_SOURCES})
        target_link_libraries(${VP_MODEL_NAME_DEBUG} PRIVATE gvsoc_debug)
        set_target_properties(${VP_MODEL_NAME_DEBUG} PROPERTIES PREFIX "")
        target_compile_options(${VP_MODEL_NAME_DEBUG} PRIVATE "-D__GVSOC__")
        target_compile_definitions(${VP_MODEL_NAME_DEBUG} PRIVATE -DGVSOC_CONFIG_EVENT_ACTIVE=1 -DVP_TRACE_ACTIVE=1 -DVP_MEMCHECK_ACTIVE=1)

        target_include_directories(${VP_MODEL_NAME_DEBUG} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

        foreach(X IN LISTS GVSOC_MODULES)
            target_include_directories(${VP_MODEL_NAME_DEBUG} PRIVATE ${X})
        endforeach()

        foreach(subdir ${VP_MODEL_INCLUDE_DIRS})
            target_include_directories(${VP_MODEL_NAME_DEBUG} PRIVATE ${subdir})
        endforeach()

        if(VP_MODEL_OUTPUT_NAME)
            set(RENAME_DEBUG_NAME ${VP_MODEL_OUTPUT_NAME})
        else()
            set(RENAME_DEBUG_NAME ${VP_MODEL_NAME_DEBUG})
        endif()

        install(
            FILES $<TARGET_FILE:${VP_MODEL_NAME_DEBUG}>
            DESTINATION  "${GVSOC_MODELS_INSTALL_FOLDER}/${GVSOC_MODELS_DEBUG_INSTALL_FOLDER}/${VP_MODEL_PREFIX}"
            RENAME "${RENAME_DEBUG_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}"
            )
    endif()

    if(${BUILD_DEBUG_M32})
        add_library(${VP_MODEL_NAME_DEBUG_M32} STATIC ${VP_MODEL_SOURCES})
        target_link_libraries(${VP_MODEL_NAME_DEBUG_M32} PRIVATE gvsoc_debug_m32)
        set_target_properties(${VP_MODEL_NAME_DEBUG_M32} PROPERTIES PREFIX "")
        target_compile_options(${VP_MODEL_NAME_DEBUG_M32} PRIVATE "-D__GVSOC__")
        target_compile_options(${VP_MODEL_NAME_DEBUG_M32} PRIVATE -m32 -D__M32_MODE__=1)
        target_link_options(${VP_MODEL_NAME_DEBUG_M32} PRIVATE -m32)
        target_compile_definitions(${VP_MODEL_NAME_DEBUG_M32} PRIVATE -DGVSOC_CONFIG_EVENT_ACTIVE=1 -DVP_TRACE_ACTIVE=1 -DVP_MEMCHECK_ACTIVE=1)

        target_include_directories(${VP_MODEL_NAME_DEBUG_M32} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

        foreach(X IN LISTS GVSOC_MODULES)
            target_include_directories(${VP_MODEL_NAME_DEBUG_M32} PRIVATE ${X})
        endforeach()

        foreach(subdir ${VP_MODEL_INCLUDE_DIRS})
            target_include_directories(${VP_MODEL_NAME_DEBUG_M32} PRIVATE ${subdir})
        endforeach()

        if(VP_MODEL_OUTPUT_NAME)
            set(RENAME_DEBUG_M32_NAME ${VP_MODEL_OUTPUT_NAME})
        else()
            set(RENAME_DEBUG_M32_NAME ${VP_MODEL_NAME_DEBUG_M32})
        endif()

        install(
            FILES $<TARGET_FILE:${VP_MODEL_NAME_DEBUG_M32}>
            DESTINATION  "${GVSOC_MODELS_INSTALL_FOLDER}/${GVSOC_MODELS_DEBUG_M32_INSTALL_FOLDER}/${VP_MODEL_PREFIX}"
            RENAME "${RENAME_DEBUG_M32_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}"
            )
    endif()

endfunction()

function(vp_block_compile_options)
    cmake_parse_arguments(
        VP_MODEL
        ""
        "NAME;FORCE_BUILD;"
        "OPTIONS"
        ${ARGN}
        )

    foreach (TARGET_TYPE IN LISTS VP_TARGET_TYPES)
        set(VP_MODEL_NAME_TYPE "${VP_MODEL_NAME}${TARGET_TYPE}")
        target_compile_options(${VP_MODEL_NAME_TYPE} PUBLIC ${VP_MODEL_OPTIONS})
    endforeach()
endfunction()

# vp_model function
function(vp_model)
    cmake_parse_arguments(
        VP_MODEL
        ""
        "NAME;PREFIX;OUTPUT_NAME;FORCE_BUILD;"
        "SOURCES;INCLUDES"
        ${ARGN}
        )
    #message(STATUS "vp_model: name=\"${VP_MODEL_NAME}\", output_name=\"${VP_MODEL_OUTPUT_NAME}\" prefix=\"${VP_MODEL_DIRECTORY}\", srcs=\"${VP_MODEL_SOURCES}\", incs=\"${VP_MODEL_INCLUDES}\"")

    # Always register metadata so child modules can declare properties even if
    # the actual target will be instantiated later by the top-level config flow.
    _vp_register_model_metadata(${VP_MODEL_NAME}
        SOURCES "${VP_MODEL_SOURCES}"
    )

    string(REPLACE "." "/" VP_MODEL_PATH ${VP_MODEL_NAME})

    get_filename_component(VP_MODEL_FILENAME ${VP_MODEL_PATH} NAME)
    get_filename_component(VP_MODEL_DIRECTORY ${VP_MODEL_PATH} DIRECTORY)

    if ("${CONFIG_${VP_MODEL_NAME}}" EQUAL "1" OR DEFINED CONFIG_BUILD_ALL OR DEFINED VP_MODEL_FORCE_BUILD)

        # TODO verify arguments
        set(VP_MODEL_NAME_OPTIM "${VP_MODEL_NAME}_optim")
        set(VP_MODEL_NAME_PROFILE "${VP_MODEL_NAME}_profile")
        set(VP_MODEL_NAME_DEBUG "${VP_MODEL_NAME}_debug")
        set(VP_MODEL_NAME_OPTIM_M32 "${VP_MODEL_NAME}_optim_m32")
        set(VP_MODEL_NAME_DEBUG_M32 "${VP_MODEL_NAME}_debug_m32")

        # ==================
        # Optimized models
        # ==================
        if(${BUILD_OPTIMIZED})
            add_library(${VP_MODEL_NAME_OPTIM} MODULE ${VP_MODEL_SOURCES})
            target_link_libraries(${VP_MODEL_NAME_OPTIM} PRIVATE gvsoc)
            set_target_properties(${VP_MODEL_NAME_OPTIM} PROPERTIES PREFIX "")
            target_compile_options(${VP_MODEL_NAME_OPTIM} PRIVATE -fno-stack-protector -D__GVSOC__)
            target_include_directories(${VP_MODEL_NAME_OPTIM} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
            foreach(X IN LISTS GVSOC_MODULES)
                target_include_directories(${VP_MODEL_NAME_OPTIM} PRIVATE ${X})
            endforeach()

            foreach(subdir ${VP_MODEL_INCLUDE_DIRS})
                target_include_directories(${VP_MODEL_NAME_OPTIM} PRIVATE ${subdir})
            endforeach()

            if(DEFINED ENV{SYSTEMC_HOME})
                target_include_directories(${VP_MODEL_NAME_OPTIM} PRIVATE $ENV{SYSTEMC_HOME}/include)
            endif()

            if(VP_MODEL_OUTPUT_NAME)
                set(RENAME_OPTIM_NAME ${VP_MODEL_OUTPUT_NAME})
            else()
                set(RENAME_OPTIM_NAME ${VP_MODEL_FILENAME})
            endif()

            install(
                FILES $<TARGET_FILE:${VP_MODEL_NAME_OPTIM}>
                DESTINATION  "${GVSOC_MODELS_INSTALL_FOLDER}/${GVSOC_MODELS_OPTIM_INSTALL_FOLDER}/${VP_MODEL_DIRECTORY}"
                RENAME "${RENAME_OPTIM_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}"
                )
        endif()

        if(${BUILD_OPTIMIZED_M32})
            add_library(${VP_MODEL_NAME_OPTIM_M32} MODULE ${VP_MODEL_SOURCES})
            target_link_libraries(${VP_MODEL_NAME_OPTIM_M32} PRIVATE gvsoc_m32)
            set_target_properties(${VP_MODEL_NAME_OPTIM_M32} PROPERTIES PREFIX "")
            target_compile_options(${VP_MODEL_NAME_OPTIM_M32} PRIVATE "-D__GVSOC__")
            target_include_directories(${VP_MODEL_NAME_OPTIM_M32} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
            target_compile_options(${VP_MODEL_NAME_OPTIM_M32} PRIVATE -m32 -D__M32_MODE__=1)
            target_link_options(${VP_MODEL_NAME_OPTIM_M32} PRIVATE -m32)
            foreach(X IN LISTS GVSOC_MODULES)
                target_include_directories(${VP_MODEL_NAME_OPTIM_M32} PRIVATE ${X})
            endforeach()

            foreach(subdir ${VP_MODEL_INCLUDE_DIRS})
                target_include_directories(${VP_MODEL_NAME_OPTIM_M32} PRIVATE ${subdir})
            endforeach()

            if(VP_MODEL_OUTPUT_NAME)
                set(RENAME_OPTIM_M32_NAME ${VP_MODEL_OUTPUT_NAME})
            else()
                set(RENAME_OPTIM_M32_NAME ${VP_MODEL_FILENAME})
            endif()

            install(
                FILES $<TARGET_FILE:${VP_MODEL_NAME_OPTIM_M32}>
                DESTINATION  "${GVSOC_MODELS_INSTALL_FOLDER}/${GVSOC_MODELS_OPTIM_M32_INSTALL_FOLDER}/${VP_MODEL_DIRECTORY}"
                RENAME "${RENAME_OPTIM_M32_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}"
                )
        endif()

        # ==================
        # Profile models
        # ==================
        if(${BUILD_PROFILE})
            add_library(${VP_MODEL_NAME_PROFILE} MODULE ${VP_MODEL_SOURCES})
            target_link_libraries(${VP_MODEL_NAME_PROFILE} PRIVATE gvsoc_profile)
            set_target_properties(${VP_MODEL_NAME_PROFILE} PROPERTIES PREFIX "")
            target_compile_options(${VP_MODEL_NAME_PROFILE} PRIVATE -fno-stack-protector -D__GVSOC__)
            # TODO VP_TRACE_ACTIVE should be removed as soon as traces have been switch to events
            target_compile_definitions(${VP_MODEL_NAME_PROFILE} PRIVATE -DCONFIG_GVSOC_EVENT_ACTIVE=1 -DVP_TRACE_ACTIVE=1)
            target_include_directories(${VP_MODEL_NAME_PROFILE} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
            foreach(X IN LISTS GVSOC_MODULES)
                target_include_directories(${VP_MODEL_NAME_PROFILE} PRIVATE ${X})
            endforeach()

            foreach(subdir ${VP_MODEL_INCLUDE_DIRS})
                target_include_directories(${VP_MODEL_NAME_PROFILE} PRIVATE ${subdir})
            endforeach()

            if(DEFINED ENV{SYSTEMC_HOME})
                target_include_directories(${VP_MODEL_NAME_PROFILE} PRIVATE $ENV{SYSTEMC_HOME}/include)
            endif()

            if(VP_MODEL_OUTPUT_NAME)
                set(RENAME_PROFILE_NAME ${VP_MODEL_OUTPUT_NAME})
            else()
                set(RENAME_PROFILE_NAME ${VP_MODEL_FILENAME})
            endif()

            install(
                FILES $<TARGET_FILE:${VP_MODEL_NAME_PROFILE}>
                DESTINATION  "${GVSOC_MODELS_INSTALL_FOLDER}/${GVSOC_MODELS_PROFILE_INSTALL_FOLDER}/${VP_MODEL_DIRECTORY}"
                RENAME "${RENAME_PROFILE_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}"
                )
        endif()

        # ==================
        # Debug models
        # ==================
        if(${BUILD_DEBUG})
            add_library(${VP_MODEL_NAME_DEBUG} MODULE ${VP_MODEL_SOURCES})
            target_link_libraries(${VP_MODEL_NAME_DEBUG} PRIVATE gvsoc_debug)
            set_target_properties(${VP_MODEL_NAME_DEBUG} PROPERTIES PREFIX "")
            target_compile_options(${VP_MODEL_NAME_DEBUG} PRIVATE "-D__GVSOC__")
            target_compile_definitions(${VP_MODEL_NAME_DEBUG} PRIVATE -DCONFIG_GVSOC_EVENT_ACTIVE=1 -DVP_TRACE_ACTIVE=1 -DVP_MEMCHECK_ACTIVE=1)
            target_include_directories(${VP_MODEL_NAME_DEBUG} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
            foreach(X IN LISTS GVSOC_MODULES)
                target_include_directories(${VP_MODEL_NAME_DEBUG} PRIVATE ${X})
            endforeach()

            foreach(subdir ${VP_MODEL_INCLUDE_DIRS})
                target_include_directories(${VP_MODEL_NAME_DEBUG} PRIVATE ${subdir})
            endforeach()

            if(DEFINED ENV{SYSTEMC_HOME})
                target_include_directories(${VP_MODEL_NAME_DEBUG} PRIVATE $ENV{SYSTEMC_HOME}/include)
            endif()

            if(VP_MODEL_OUTPUT_NAME)
                set(RENAME_DEBUG_NAME ${VP_MODEL_OUTPUT_NAME})
            else()
                set(RENAME_DEBUG_NAME ${VP_MODEL_FILENAME})
            endif()

            install(
                FILES $<TARGET_FILE:${VP_MODEL_NAME_DEBUG}>
                DESTINATION  "${GVSOC_MODELS_INSTALL_FOLDER}/${GVSOC_MODELS_DEBUG_INSTALL_FOLDER}/${VP_MODEL_DIRECTORY}"
                RENAME "${RENAME_DEBUG_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}"
                )
        endif()

        if(${BUILD_DEBUG_M32})
            add_library(${VP_MODEL_NAME_DEBUG_M32} MODULE ${VP_MODEL_SOURCES})
            target_link_libraries(${VP_MODEL_NAME_DEBUG_M32} PRIVATE gvsoc_debug_m32)
            set_target_properties(${VP_MODEL_NAME_DEBUG_M32} PROPERTIES PREFIX "")
            target_compile_options(${VP_MODEL_NAME_DEBUG_M32} PRIVATE "-D__GVSOC__")
            target_compile_options(${VP_MODEL_NAME_DEBUG_M32} PRIVATE -m32 -D__M32_MODE__=1)
            target_link_options(${VP_MODEL_NAME_DEBUG_M32} PRIVATE -m32)
            target_compile_definitions(${VP_MODEL_NAME_DEBUG_M32} PRIVATE -DVP_TRACE_ACTIVE=1 -DVP_MEMCHECK_ACTIVE=1)
            target_include_directories(${VP_MODEL_NAME_DEBUG_M32} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
            foreach(X IN LISTS GVSOC_MODULES)
                target_include_directories(${VP_MODEL_NAME_DEBUG_M32} PRIVATE ${X})
            endforeach()

            foreach(subdir ${VP_MODEL_INCLUDE_DIRS})
                target_include_directories(${VP_MODEL_NAME_DEBUG_M32} PRIVATE ${subdir})
            endforeach()

            if(VP_MODEL_OUTPUT_NAME)
                set(RENAME_DEBUG_M32_NAME ${VP_MODEL_OUTPUT_NAME})
            else()
                set(RENAME_DEBUG_M32_NAME ${VP_MODEL_FILENAME})
            endif()

            install(
                FILES $<TARGET_FILE:${VP_MODEL_NAME_DEBUG_M32}>
                DESTINATION  "${GVSOC_MODELS_INSTALL_FOLDER}/${GVSOC_MODELS_DEBUG_M32_INSTALL_FOLDER}/${VP_MODEL_DIRECTORY}"
                RENAME "${RENAME_DEBUG_M32_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}"
                )
        endif()

        if (NOT "${CONFIG_CFLAGS_${VP_MODEL_NAME}}" STREQUAL "")
            foreach (TARGET_TYPE IN LISTS VP_TARGET_TYPES)
                set(VP_MODEL_NAME_TYPE "${VP_MODEL_NAME}${TARGET_TYPE}")
                string(REPLACE " " ";" CFLAGS_LIST "${CONFIG_CFLAGS_${VP_MODEL_NAME}}")
                foreach (CFLAG IN LISTS CFLAGS_LIST)
                    target_compile_options(${VP_MODEL_NAME_TYPE} PRIVATE ${CFLAG})
                endforeach()
            endforeach()
        endif()

        # Inherit metadata declared by child modules which registered the same
        # model sources under a logical name.
        _vp_inherit_registered_metadata(${VP_MODEL_NAME}
            SOURCES "${VP_MODEL_SOURCES}"
        )

    endif()
endfunction()

function(vp_model_link_gvsoc)
    cmake_parse_arguments(
        VP_MODEL
        ""
        "NAME;FORCE_BUILD;"
        "LIBRARY"
        ${ARGN}
        )

    if ("${CONFIG_${VP_MODEL_NAME}}" EQUAL "1" OR DEFINED CONFIG_BUILD_ALL OR DEFINED VP_MODEL_FORCE_BUILD)
        foreach (TARGET_TYPE IN LISTS VP_TARGET_TYPES)
            set(VP_MODEL_NAME_TARGET "${VP_MODEL_NAME}${TARGET_TYPE}")
            target_link_libraries(${VP_MODEL_NAME_TARGET} PRIVATE gvsoc${TARGET_TYPE})
        endforeach()
    endif()
endfunction()

function(vp_block_link_libraries)
    cmake_parse_arguments(
        VP_MODEL
        ""
        "NAME;NO_M32;"
        "LIBRARY"
        ${ARGN}
    )

    if(TARGET_TYPES)
    else()
        set(TARGET_TYPES ${VP_TARGET_TYPES})
    endif()
    foreach (TARGET_TYPE IN LISTS VP_TARGET_TYPES)
        if (VP_MODEL_NO_M32 AND (TARGET_TYPE STREQUAL _debug_m32 OR TARGET_TYPE STREQUAL _optim_m32))
        else()
            set(VP_MODEL_NAME_TARGET "${VP_MODEL_NAME}${TARGET_TYPE}")
            target_link_libraries(${VP_MODEL_NAME_TARGET} PRIVATE ${VP_MODEL_LIBRARY})
        endif()
    endforeach()
endfunction()

function(vp_model_link_libraries)
    cmake_parse_arguments(
        VP_MODEL
        ""
        "NAME;NO_M32;FORCE_BUILD;"
        "LIBRARY"
        ${ARGN}
        )

    # Always register metadata
    _vp_register_model_metadata(${VP_MODEL_NAME}
        LINK_LIBS "${VP_MODEL_LIBRARY}"
    )

    if(TARGET_TYPES)
    else()
        set(TARGET_TYPES ${VP_TARGET_TYPES})
    endif()

    if ("${CONFIG_${VP_MODEL_NAME}}" EQUAL "1" OR DEFINED CONFIG_BUILD_ALL OR DEFINED VP_MODEL_FORCE_BUILD)
        foreach (TARGET_TYPE IN LISTS VP_TARGET_TYPES)
            if (VP_MODEL_NO_M32 AND (TARGET_TYPE STREQUAL _debug_m32 OR TARGET_TYPE STREQUAL _optim_m32))
            else()
                set(VP_MODEL_NAME_TARGET "${VP_MODEL_NAME}${TARGET_TYPE}")
                target_link_libraries(${VP_MODEL_NAME_TARGET} PRIVATE ${VP_MODEL_LIBRARY})
            endif()
        endforeach()
    endif()
endfunction()

function(vp_model_link_blocks)
    cmake_parse_arguments(
        VP_MODEL
        ""
        "NAME;FORCE_BUILD;"
        "BLOCK"
        ${ARGN}
        )

    if ("${CONFIG_${VP_MODEL_NAME}}" EQUAL "1" OR DEFINED CONFIG_BUILD_ALL OR DEFINED VP_MODEL_FORCE_BUILD)
        foreach (TARGET_TYPE IN LISTS VP_TARGET_TYPES)
            set(VP_MODEL_NAME_TARGET "${VP_MODEL_NAME}${TARGET_TYPE}")
            target_link_libraries(${VP_MODEL_NAME_TARGET} PRIVATE ${VP_MODEL_BLOCK}${TARGET_TYPE})
        endforeach()
    endif()
endfunction()

function(vp_model_compile_options)
    cmake_parse_arguments(
        VP_MODEL
        ""
        "NAME;FORCE_BUILD;"
        "OPTIONS"
        ${ARGN}
        )

    # Always register metadata
    _vp_register_model_metadata(${VP_MODEL_NAME}
        COMPILE_OPTIONS "${VP_MODEL_OPTIONS}"
    )

    if ("${CONFIG_${VP_MODEL_NAME}}" EQUAL "1" OR DEFINED CONFIG_BUILD_ALL OR DEFINED VP_MODEL_FORCE_BUILD)
        foreach (TARGET_TYPE IN LISTS VP_TARGET_TYPES)
            set(VP_MODEL_NAME_TYPE "${VP_MODEL_NAME}${TARGET_TYPE}")
            target_compile_options(${VP_MODEL_NAME_TYPE} PRIVATE ${VP_MODEL_OPTIONS})
        endforeach()
    endif()
endfunction()

function(vp_model_link_options)
    cmake_parse_arguments(
        VP_MODEL
        ""
        "NAME;FORCE_BUILD;"
        "OPTIONS"
        ${ARGN}
        )

    # Always register metadata
    _vp_register_model_metadata(${VP_MODEL_NAME}
        LINK_OPTIONS "${VP_MODEL_OPTIONS}"
    )

    if ("${CONFIG_${VP_MODEL_NAME}}" EQUAL "1" OR DEFINED CONFIG_BUILD_ALL OR DEFINED VP_MODEL_FORCE_BUILD)
        foreach (TARGET_TYPE IN LISTS VP_TARGET_TYPES)
            set(VP_MODEL_NAME_TYPE "${VP_MODEL_NAME}${TARGET_TYPE}")
            target_link_options(${VP_MODEL_NAME_TYPE} PRIVATE ${VP_MODEL_OPTIONS})
        endforeach()
    endif()
endfunction()

function(vp_model_compile_definitions)
    cmake_parse_arguments(
        VP_MODEL
        ""
        "NAME;FORCE_BUILD;"
        "DEFINITIONS"
        ${ARGN}
        )

    # Always register metadata
    _vp_register_model_metadata(${VP_MODEL_NAME}
        COMPILE_DEFINITIONS "${VP_MODEL_DEFINITIONS}"
    )

    if ("${CONFIG_${VP_MODEL_NAME}}" EQUAL "1" OR DEFINED CONFIG_BUILD_ALL OR DEFINED VP_MODEL_FORCE_BUILD)
        foreach (TARGET_TYPE IN LISTS VP_TARGET_TYPES)
            set(VP_MODEL_NAME_TYPE "${VP_MODEL_NAME}${TARGET_TYPE}")
            target_compile_definitions(${VP_MODEL_NAME_TYPE} PRIVATE ${VP_MODEL_DEFINITIONS})
        endforeach()
    endif()
endfunction()

function(vp_model_include_directories)
    cmake_parse_arguments(
        VP_MODEL
        ""
        "NAME;FORCE_BUILD;"
        "DIRECTORY"
        ${ARGN}
        )

    # Always register metadata
    _vp_register_model_metadata(${VP_MODEL_NAME}
        INCLUDE_DIRS "${VP_MODEL_DIRECTORY}"
    )

    if ("${CONFIG_${VP_MODEL_NAME}}" EQUAL "1" OR DEFINED CONFIG_BUILD_ALL OR DEFINED VP_MODEL_FORCE_BUILD)
        foreach (TARGET_TYPE IN LISTS VP_TARGET_TYPES)
            set(VP_MODEL_NAME_TYPE "${VP_MODEL_NAME}${TARGET_TYPE}")
            target_include_directories(${VP_MODEL_NAME_TYPE} PRIVATE ${VP_MODEL_DIRECTORY})
        endforeach()
    endif()
endfunction()

function(vp_model_sources)
    cmake_parse_arguments(
        VP_MODEL
        ""
        "NAME;FORCE_BUILD;"
        "SOURCES"
        ${ARGN}
        )

    # Always register metadata
    _vp_register_model_metadata(${VP_MODEL_NAME}
        SOURCES "${VP_MODEL_SOURCES}"
    )

    if ("${CONFIG_${VP_MODEL_NAME}}" EQUAL "1" OR DEFINED CONFIG_BUILD_ALL OR DEFINED VP_MODEL_FORCE_BUILD)
        foreach (TARGET_TYPE IN LISTS VP_TARGET_TYPES)
            set(VP_MODEL_NAME_TYPE "${VP_MODEL_NAME}${TARGET_TYPE}")
            target_sources(${VP_MODEL_NAME_TYPE} PRIVATE ${VP_MODEL_SOURCES})
        endforeach()
    endif()
endfunction()

function(vp_files)
    cmake_parse_arguments(
        VP_FILES
        ""
        "PREFIX"
        "FILES"
        ${ARGN}
        )
    #message(STATUS "vp_files: prefix=\"${VP_FILES_PREFIX}\", files=\"${VP_FILES_FILES}\"")
    install(FILES ${VP_FILES_FILES}
        DESTINATION "generators/${VP_FILES_PREFIX}")
endfunction()

function(vp_precompiled_models)
    cmake_parse_arguments(
        VP_FILES
        ""
        "PREFIX"
        "FILES"
        ${ARGN}
        )
    #message(STATUS "vp_files: prefix=\"${VP_FILES_PREFIX}\", files=\"${VP_FILES_FILES}\"")
    install(FILES ${VP_FILES_FILES}
        DESTINATION "models/${VP_FILES_PREFIX}")
endfunction()