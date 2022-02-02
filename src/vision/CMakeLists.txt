cmake_minimum_required(VERSION 3.12)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/object)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/detector)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/predictor)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/classifier)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/compensator)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/params)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/process)