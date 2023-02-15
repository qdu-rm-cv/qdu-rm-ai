cmake_minimum_required(VERSION 3.12)
project(module_component)

configure_file(build_type.hpp.in ${CMAKE_BINARY_DIR}/generated/build_type.hpp)

add_subdirectory(${ROOT_PATH}/third_party/OneMessageCPP om.out)

file(GLOB ${PROJECT_NAME}_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

target_link_libraries(${PROJECT_NAME} PUBLIC spdlog::spdlog)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/generated/
)

include_files(${CMAKE_CURRENT_SOURCE_DIR})
include_files(${CMAKE_BINARY_DIR}/generated)
lib_install(${PROJECT_NAME})
