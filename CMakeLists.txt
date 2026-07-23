cmake_minimum_required(VERSION 3.16)
project(Beyondmimic_Deploy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# A Debug default is easy to select accidentally on the robot and is not
# appropriate for a 50 Hz inference loop.  Multi-config generators (for
# example Ninja Multi-Config) choose the configuration at build time.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" DEPLOY_REAL_TARGET_PROCESSOR)
if(DEPLOY_REAL_TARGET_PROCESSOR MATCHES "^(x86_64|amd64)$")
    set(DEPLOY_REAL_ONNXRUNTIME_ARCH "x64")
elseif(DEPLOY_REAL_TARGET_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(DEPLOY_REAL_ONNXRUNTIME_ARCH "aarch64")
else()
    message(FATAL_ERROR
        "Unsupported target processor '${CMAKE_SYSTEM_PROCESSOR}'. "
        "Use an x86_64 or aarch64 toolchain.")
endif()

# The G1 PC2 computer is aarch64.  Keep the runtime outside source control and
# select the matching package automatically; callers may always override this
# cache variable, e.g. -DONNXRUNTIME_ROOT=/opt/onnxruntime-linux-aarch64-1.22.0.
set(ONNXRUNTIME_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/onnxruntime-linux-${DEPLOY_REAL_ONNXRUNTIME_ARCH}-1.22.0"
    CACHE PATH "Path to the ONNX Runtime distribution")
# find_path/find_file cache their result.  Clear only these derived values so
# changing ONNXRUNTIME_ROOT between x64 and aarch64 reconfigures correctly.
unset(ONNXRUNTIME_INCLUDE_DIR CACHE)
unset(ONNXRUNTIME_LIBRARY CACHE)
find_path(ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    HINTS "${ONNXRUNTIME_ROOT}/include"
    NO_DEFAULT_PATH)
find_file(ONNXRUNTIME_LIBRARY
    NAMES libonnxruntime.so libonnxruntime.so.1 libonnxruntime.so.1.22.0
    HINTS "${ONNXRUNTIME_ROOT}/lib" "${ONNXRUNTIME_ROOT}/lib64"
    NO_DEFAULT_PATH)

if(NOT ONNXRUNTIME_INCLUDE_DIR)
    message(FATAL_ERROR
        "ONNX Runtime headers were not found under '${ONNXRUNTIME_ROOT}'. "
        "Install the ${DEPLOY_REAL_ONNXRUNTIME_ARCH} package described in thirdparty/README.md "
        "or set -DONNXRUNTIME_ROOT=<path>.")
endif()
if(NOT ONNXRUNTIME_LIBRARY)
    message(FATAL_ERROR
        "libonnxruntime.so was not found below '${ONNXRUNTIME_ROOT}/lib' or lib64. "
        "Use an ONNX Runtime 1.22.0 package built for ${DEPLOY_REAL_ONNXRUNTIME_ARCH}.")
endif()

add_library(onnxruntime::onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
)

get_filename_component(ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
set(CMAKE_BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}")
set(CMAKE_INSTALL_RPATH "${ONNXRUNTIME_LIBRARY_DIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(unitree_sdk2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

add_library(deploy_control STATIC
    core/ControlTypes.cpp
    core/SafetySupervisor.cpp
    core/StateMachine.cpp
    inference/OnnxRuntime.cpp
    states/StateFactory.cpp
    states/basic/BasicStates.cpp
    states/loco/LocoConfig.cpp
    states/loco/LocoState.cpp
    states/policy/PolicyState.cpp
    states/redred/RedRedConfig.cpp
    states/redred/RedRedCastState.cpp
    states/redred/RedRedState.cpp
    states/startup/StartupReadyState.cpp
)
target_include_directories(deploy_control PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(deploy_control PUBLIC
    DEPLOY_REAL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(deploy_control PUBLIC
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
    onnxruntime::onnxruntime
)
# ONNX Runtime keeps a stable SONAME across releases.  Prefer the runtime named
# by ONNXRUNTIME_ROOT even when a ROS environment prepends an older one to
# LD_LIBRARY_PATH; otherwise a binary linked to 1.22 can load an incompatible
# libonnxruntime.so.1 at runtime.
if(UNIX AND NOT APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_options(deploy_control INTERFACE "-Wl,--disable-new-dtags")
endif()

add_executable(Beyondmimic_Deploy
    app/main.cpp
    hardware/unitree/Crc.cpp
    hardware/unitree/UnitreeController.cpp
)
target_link_libraries(Beyondmimic_Deploy PRIVATE
    deploy_control
    unitree_sdk2
    Threads::Threads
)
