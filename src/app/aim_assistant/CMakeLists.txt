cmake_minimum_required(VERSION 3.12)
project(exec_aim_assistant)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
    module_behavior
    module_compensator
    module_process
    spdlog::spdlog
    ${Dcamera}
    ${Dcontroller}
    ${Taim}_detector
    ${Tbuff}_detector
    ${Taim}_object
    ${Tbuff}_object
    ${Taim}_predictor
    ${Tbuff}_predictor
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
    $<TARGET_PROPERTY:module_behavior,INTERFACE_INCLUDE_DIRECTORIES>
)

add_dependencies(${PROJECT_NAME}
    ${Dcamera}
    ${Dcontroller}
)