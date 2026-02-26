cmake_minimum_required(VERSION 3.20)
project(aero_lbm LANGUAGES C CXX)

option(AERO_LBM_SUPERBUILD "Build all platform binaries in one invocation" OFF)
set(AERO_LBM_DIST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dist/natives" CACHE PATH "Directory used by multi-platform superbuild outputs")
set(AERO_LBM_OUTPUT_DIR "" CACHE PATH "Optional output directory for single-target builds")
set(AERO_LBM_BUILD_TYPE_RELEASE "Release" CACHE STRING "Build type used by AERO_LBM_SUPERBUILD")
set_property(CACHE AERO_LBM_BUILD_TYPE_RELEASE PROPERTY STRINGS Release RelWithDebInfo MinSizeRel Debug)

if (AERO_LBM_SUPERBUILD)
    include(ExternalProject)

    file(MAKE_DIRECTORY "${AERO_LBM_DIST_DIR}")
    string(TOUPPER "${AERO_LBM_BUILD_TYPE_RELEASE}" AERO_LBM_BUILD_TYPE_UPPER)

    function(aero_add_platform build_id output_subdir toolchain_rel output_name)
        set(toolchain_file "${CMAKE_CURRENT_SOURCE_DIR}/${toolchain_rel}")
        if (NOT EXISTS "${toolchain_file}")
            message(FATAL_ERROR "Missing toolchain file: ${toolchain_file}")
        endif()

        ExternalProject_Add("aero_lbm_${build_id}"
            SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
            BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/build-${build_id}"
            CMAKE_ARGS
                -DAERO_LBM_SUPERBUILD=OFF
                -DAERO_LBM_ENABLE_OPENCL=OFF
                -DAERO_LBM_OUTPUT_DIR=${AERO_LBM_DIST_DIR}/${output_subdir}
                -DCMAKE_BUILD_TYPE=${AERO_LBM_BUILD_TYPE_RELEASE}
                -DCMAKE_LIBRARY_OUTPUT_DIRECTORY_${AERO_LBM_BUILD_TYPE_UPPER}=${AERO_LBM_DIST_DIR}/${output_subdir}
                -DCMAKE_RUNTIME_OUTPUT_DIRECTORY_${AERO_LBM_BUILD_TYPE_UPPER}=${AERO_LBM_DIST_DIR}/${output_subdir}
                -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_${AERO_LBM_BUILD_TYPE_UPPER}=${AERO_LBM_DIST_DIR}/${output_subdir}
                -DCMAKE_TOOLCHAIN_FILE=${toolchain_file}
            BUILD_COMMAND "${CMAKE_COMMAND}" --build <BINARY_DIR> --config "${AERO_LBM_BUILD_TYPE_RELEASE}"
            BUILD_BYPRODUCTS "${AERO_LBM_DIST_DIR}/${output_subdir}/${output_name}"
            INSTALL_COMMAND ""
        )
    endfunction()

    aero_add_platform("linux_x86_64" "linux-x86_64" "cmake/toolchains/linux-x86_64.cmake" "libaero_lbm.so")
    aero_add_platform("linux_arm64" "linux-arm64" "cmake/toolchains/linux-arm64.cmake" "libaero_lbm.so")
    aero_add_platform("windows_x86_64" "windows-x86_64" "cmake/toolchains/windows-x86_64.cmake" "aero_lbm.dll")
    aero_add_platform("macos_x86_64" "macos-x86_64" "cmake/toolchains/macos-x86_64.cmake" "libaero_lbm.dylib")
    aero_add_platform("macos_arm64" "macos-arm64" "cmake/toolchains/macos-arm64.cmake" "libaero_lbm.dylib")

    add_custom_target(aero_lbm_all ALL
        DEPENDS
            aero_lbm_linux_x86_64
            aero_lbm_linux_arm64
            aero_lbm_windows_x86_64
            aero_lbm_macos_x86_64
            aero_lbm_macos_arm64
    )

    message(STATUS "AERO_LBM_SUPERBUILD enabled")
    message(STATUS "Output layout: ${AERO_LBM_DIST_DIR}/<platform>/<library>")
    return()
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(JNI REQUIRED)

set(_opencl_default ON)
if (CMAKE_CROSSCOMPILING)
    set(_opencl_default OFF)
endif()
option(AERO_LBM_ENABLE_OPENCL "Enable OpenCL backend in native solver" ${_opencl_default})

add_library(aero_lbm SHARED
    src/aero_lbm_jni.cpp
)

target_include_directories(aero_lbm PRIVATE ${JNI_INCLUDE_DIRS})

if (AERO_LBM_ENABLE_OPENCL)
    find_package(OpenCL QUIET)
    if (OpenCL_FOUND)
        target_compile_definitions(aero_lbm PRIVATE AERO_LBM_OPENCL=1)
        target_link_libraries(aero_lbm PRIVATE OpenCL::OpenCL)
    endif()
endif()

if (WIN32)
    set_target_properties(aero_lbm PROPERTIES PREFIX "")
endif()

if (AERO_LBM_OUTPUT_DIR)
    file(MAKE_DIRECTORY "${AERO_LBM_OUTPUT_DIR}")
    set_target_properties(aero_lbm PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${AERO_LBM_OUTPUT_DIR}"
        RUNTIME_OUTPUT_DIRECTORY "${AERO_LBM_OUTPUT_DIR}"
        ARCHIVE_OUTPUT_DIRECTORY "${AERO_LBM_OUTPUT_DIR}"
    )
endif()

if (MSVC)
    target_compile_options(aero_lbm PRIVATE /W4)
else()
    target_compile_options(aero_lbm PRIVATE -Wall -Wextra -Wpedantic)
endif()

# TODO:
# - Add mixed precision storage (fp16 / int16) with fp32 accumulation.
# - Add kernel-side boundary variants (slip/inflow/outflow tuning).
