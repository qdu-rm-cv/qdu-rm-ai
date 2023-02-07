cmake_minimum_required(VERSION 3.12)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/object)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/detector)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/predictor)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/classifier)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/compensator)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/params)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/process)

include_files(${CMAKE_CURRENT_SOURCE_DIR}/object)
include_files(${CMAKE_CURRENT_SOURCE_DIR}/detector)
include_files(${CMAKE_CURRENT_SOURCE_DIR}/predictor)
include_files(${CMAKE_CURRENT_SOURCE_DIR}/classifier)
include_files(${CMAKE_CURRENT_SOURCE_DIR}/compensator)
include_files(${CMAKE_CURRENT_SOURCE_DIR}/params)
include_files(${CMAKE_CURRENT_SOURCE_DIR}/process)

lib_install(module_object_base)
lib_install(module_classifier)
lib_install(module_predictor_base)
lib_install(module_compensator)
lib_install(module_process)
lib_install(module_param)

if(BUILD_NN)
    lib_install(${Tradar}_detector)
endif()

lib_install(${Taim}_object)
lib_install(${Taim}_detector)
lib_install(${Taim}_predictor)

lib_install(${Tbuff}_object)
lib_install(${Tbuff}_detector)
lib_install(${Tbuff}_predictor)

lib_install(${Tdart}_object)
lib_install(${Tdart}_detector)

lib_install(${Tengineer}_object)
