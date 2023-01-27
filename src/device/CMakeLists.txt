cmake_minimum_required(VERSION 3.12)
project(device)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/camera)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/controller)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/servo)
