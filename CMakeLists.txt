if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
    message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there.")
endif()

cmake_minimum_required(VERSION 3.12)

project(qdu-rm-ai
    DESCRIPTION "QDU AI for RoboMaster"
    VERSION 0.1.0
    LANGUAGES CXX
)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# ---------------------------------------------------------------------------------------
# Platform
# ---------------------------------------------------------------------------------------
message(STATUS "------------------------- Platform ------------------------------")

string(TIMESTAMP CONFIG_TIMESTAMP "" UTC)
message(STATUS "Timestamp: ${CONFIG_TIMESTAMP}")
message(STATUS "Host: ${CMAKE_HOST_SYSTEM_NAME} ${CMAKE_HOST_SYSTEM_VERSION} ${CMAKE_HOST_SYSTEM_PROCESSOR}")

if(CMAKE_CROSSCOMPILING)
    message(STATUS "Target: ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_VERSION} ${CMAKE_SYSTEM_PROCESSOR}")
endif()

message(STATUS "CMake: ${CMAKE_VERSION}")
message(STATUS "CMake generator: ${CMAKE_GENERATOR}")
message(STATUS "CMake build tool: ${CMAKE_BUILD_TOOL}")

if(MSVC)
    message(STATUS "MSVC: ${MSVC_VERSION}")
endif()

if(CMAKE_GENERATOR MATCHES Xcode)
    message(STATUS "Xcode: ${XCODE_VERSION}")
endif()

if(NOT CMAKE_GENERATOR MATCHES "Xcode|Visual Studio")
    message(STATUS "Configuration: ${CMAKE_BUILD_TYPE}")
endif()

# ---------------------------------------------------------------------------------------
# Compiler Options
# ---------------------------------------------------------------------------------------
message(STATUS "-------------------------- Compiler ------------------------------")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "C++ flags (Release): ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "C++ flags (Debug): ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")

if(WIN32)
    message(STATUS "Linker flags (Release): ${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_RELEASE}")
    message(STATUS "Linker flags (Debug): ${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_DEBUG}")
else()
    message(STATUS "Linker flags (Release): ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")
    message(STATUS "Linker flags (Debug): ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_DEBUG}")
endif()

# ---------------------------------------------------------------------------------------
# Building Options
# ---------------------------------------------------------------------------------------
message(STATUS "---------------------- Building Options ------------------------")

find_package(behaviortree_cpp_v3 REQUIRED)
find_package(Ceres 2 REQUIRED)
find_package(CUDA)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)
find_package(TBB REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(usbp REQUIRED libusbp-1)

message(STATUS "BehaviorTreeV3 version: ${BehaviorTreeV3_VERSION}")
message(STATUS "Ceres version: ${Ceres_VERSION}")
message(STATUS "CUDA version: ${CUDA_VERSION}")
message(STATUS "Eigen version: ${Eigen3_VERSION}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "spdlog version: ${spdlog_VERSION}")
message(STATUS "TBB version: ${TBB_VERSION}")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(BUILD_TYPE LEVEL_DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    set(BUILD_TYPE LEVEL_DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(BUILD_TYPE LEVEL_INFO)
elseif(CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
    set(BUILD_TYPE LEVEL_INFO)
endif()

# ---------------------------------------------------------------------------------------
# Packages Option
# ---------------------------------------------------------------------------------------
message(STATUS "----------------------- Packages Option ------------------------")

option(BUILD_NN "build nn module" OFF)
option(WITH_DEVICE "No device" OFF)
option(WITH_UI "No UI" OFF)
set(ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

if(BUILD_NN)
    set(BUILD_NN 1)
else()
    set(BUILD_NN 0)
endif()

if(WITH_DEVICE)
    set(WITH_DEVICE 1)
else()
    set(WITH_DEVICE 0)
endif()

if(WITH_UI)
    set(WITH_UI 1)
else()
    set(WITH_UI 0)
endif()

message(STATUS "Build NN: ${BUILD_NN}")
message(STATUS "WITH_DEVICE: ${WITH_DEVICE}")
message(STATUS "WITH_UI: ${WITH_UI}")

# ---------------------------------------------------------------------------------------
# Source
# ---------------------------------------------------------------------------------------
message(STATUS "--------------------------- Source -----------------------------")

set(Taim "task_aim")
set(Tbuff "task_buff")
set(Tdart "task_dart")
set(Tengineer "task_engineer")
set(Tsentry "task_sentry")
set(Tradar "task_radar")
message(STATUS "Build task: ${Taim}")
message(STATUS "Build task: ${Tbuff}")
message(STATUS "Build task: ${Tdart}")
message(STATUS "Build task: ${Tengineer}")
message(STATUS "Build task: ${Tsentry}")
message(STATUS "Build task: ${Tradar}")

set(Dcamera "device_camera")
set(Dcontroller "device_controller")
set(Dservo "device_servo")
message(STATUS "Build device: ${Dcamera}")
message(STATUS "Build device: ${Dcontroller}")
message(STATUS "Build device: ${Dservo}")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

# ---------------------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------------------
message(STATUS "---------------------------- Test -------------------------------")

enable_testing()
find_package(GTest REQUIRED)
message(STATUS "Google test version: ${GTest_VERSION}")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tests)

# ---------------------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------------------
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_INSTALL_LIBDIR /usr/local/lib/RM)
    set(CMAKE_INSTALL_BINDIR /usr/local/bin/RM)
    message(STATUS "------------------------ Install --------------------------------")
    message(STATUS "Generating install")
    message(STATUS "Install to: ${CMAKE_INSTALL_PREFIX}")
    message(STATUS "Install to: ${CMAKE_INSTALL_LIBDIR}")
    message(STATUS "Install to: ${CMAKE_INSTALL_BINDIR}")

    install(
        TARGETS exec_auto_aim
        EXPORT auto-aim
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

    install(
        TARGETS exec_sentry
        EXPORT sentry
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

    install(
        TARGETS exec_buff
        EXPORT buff
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

message(STATUS "-----------------------------------------------------------------")
