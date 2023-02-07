cmake_minimum_required(VERSION 3.12)
project(device)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/camera)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/controller)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/servo)

include_files(${CMAKE_CURRENT_SOURCE_DIR}/camera)
include_files(${CMAKE_CURRENT_SOURCE_DIR}/controller)
include_files(${CMAKE_CURRENT_SOURCE_DIR}/servo)

lib_install(${Dcamera})
lib_install(${Dcontroller})
lib_install(${Dservo})