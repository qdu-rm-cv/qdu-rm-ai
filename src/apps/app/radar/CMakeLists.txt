cmake_minimum_required(VERSION 3.12)
project(exec_radar)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
    module_behavior
    module_compensator
    spdlog::spdlog
    ${Dcamera}
    ${Dcontroller}
    ${Tradar}_detector
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(${PROJECT_NAME}
    ${Dcamera}
    ${Dcontroller}
)