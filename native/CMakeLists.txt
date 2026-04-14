cmake_minimum_required(VERSION 3.20)
project(aero_lbm LANGUAGES C CXX)

option(AERO_LBM_SUPERBUILD "Build all platform binaries in one invocation" OFF)
set(AERO_LBM_DIST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dist/natives" CACHE PATH "Directory used by multi-platform superbuild outputs")
set(AERO_LBM_OUTPUT_DIR "" CACHE PATH "Optional output directory for single-target builds")
set(AERO_LBM_BUILD_TYPE_RELEASE "Release" CACHE STRING "Build type used by AERO_LBM_SUPERBUILD")
set(AERO_LBM_SUPERBUILD_ENABLE_OPENCL ON CACHE BOOL "Build superbuild targets with OpenCL enabled")
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
                -DAERO_LBM_ENABLE_OPENCL=${AERO_LBM_SUPERBUILD_ENABLE_OPENCL}
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
    aero_add_platform("macos_arm64" "macos-arm64" "cmake/toolchains/macos-arm64.cmake" "libaero_lbm.dylib")

    add_custom_target(aero_lbm_all ALL
        DEPENDS
            aero_lbm_linux_x86_64
            aero_lbm_linux_arm64
            aero_lbm_windows_x86_64
            aero_lbm_macos_arm64
    )

    message(STATUS "AERO_LBM_SUPERBUILD enabled")
    message(STATUS "Output layout: ${AERO_LBM_DIST_DIR}/<platform>/<library>")
    return()
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(AERO_LBM_WINDOWS_JAVA_HOME "" CACHE PATH "Path to a Windows JDK used for JNI headers when cross-compiling aero_lbm.dll from Linux/WSL2")
set(AERO_LBM_WINDOWS_JNI_INCLUDE_DIR "" CACHE PATH "Override path to the generic JNI include directory for Windows cross builds")
set(AERO_LBM_WINDOWS_JNI_PLATFORM_INCLUDE_DIR "" CACHE PATH "Override path to the Windows-specific JNI include directory for Windows cross builds")

if (WIN32 AND CMAKE_CROSSCOMPILING)
    set(_aero_windows_jni_include_dir "${AERO_LBM_WINDOWS_JNI_INCLUDE_DIR}")
    set(_aero_windows_jni_platform_include_dir "${AERO_LBM_WINDOWS_JNI_PLATFORM_INCLUDE_DIR}")

    if (NOT _aero_windows_jni_include_dir OR NOT _aero_windows_jni_platform_include_dir)
        set(_aero_windows_java_home "${AERO_LBM_WINDOWS_JAVA_HOME}")
        if (NOT _aero_windows_java_home AND DEFINED ENV{AERO_WINDOWS_JAVA_HOME})
            set(_aero_windows_java_home "$ENV{AERO_WINDOWS_JAVA_HOME}")
        endif()
        if (_aero_windows_java_home)
            file(TO_CMAKE_PATH "${_aero_windows_java_home}" _aero_windows_java_home)
            if (NOT _aero_windows_jni_include_dir)
                set(_aero_windows_jni_include_dir "${_aero_windows_java_home}/include")
            endif()
            if (NOT _aero_windows_jni_platform_include_dir)
                set(_aero_windows_jni_platform_include_dir "${_aero_windows_java_home}/include/win32")
            endif()
        endif()
    endif()

    if (EXISTS "${_aero_windows_jni_include_dir}/jni.h" AND EXISTS "${_aero_windows_jni_platform_include_dir}/jni_md.h")
        set(JNI_FOUND TRUE)
        set(JNI_INCLUDE_DIRS
            "${_aero_windows_jni_include_dir}"
            "${_aero_windows_jni_platform_include_dir}"
        )
        message(STATUS "Using Windows JNI headers for cross build: ${_aero_windows_jni_include_dir} ; ${_aero_windows_jni_platform_include_dir}")
    else()
        message(FATAL_ERROR
            "Windows x86_64 cross-build requires Windows JNI headers.\n"
            "Set AERO_LBM_WINDOWS_JAVA_HOME to a Windows JDK root (for example /mnt/c/Program Files/Java/jdk-21),\n"
            "or provide AERO_LBM_WINDOWS_JNI_INCLUDE_DIR and AERO_LBM_WINDOWS_JNI_PLATFORM_INCLUDE_DIR explicitly."
        )
    endif()
else()
    find_package(JNI REQUIRED)
endif()

option(AERO_LBM_ENABLE_OPENCL "Enable OpenCL backend in native solver" ON)

add_library(aero_lbm SHARED
    src/aero_lbm_jni.cpp
    src/aero_lbm_analysis_codec.cpp
    src/aero_lbm_hydro_core.cpp
    src/aero_lbm_thermal_core.cpp
    src/aero_lbm_mesoscale.cpp
    src/aero_lbm_simulation_bridge.cpp
)

add_subdirectory(third_party/zfp)

target_include_directories(aero_lbm PRIVATE ${JNI_INCLUDE_DIRS})
target_include_directories(aero_lbm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if (AERO_LBM_ENABLE_OPENCL)
    find_package(OpenCL REQUIRED)
    target_compile_definitions(aero_lbm PRIVATE AERO_LBM_OPENCL=1)
    target_link_libraries(aero_lbm PRIVATE OpenCL::OpenCL)
else()
    message(STATUS "AERO_LBM_ENABLE_OPENCL=OFF (CPU-only native build)")
endif()

target_link_libraries(aero_lbm PRIVATE zfp::zfp)

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
