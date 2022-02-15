cmake_minimum_required(VERSION 3.12)
project(ui-param)

add_executable(${PROJECT_NAME} guiding_light.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
    device
    param
    detector
    spdlog::spdlog
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(${PROJECT_NAME}
    device
)