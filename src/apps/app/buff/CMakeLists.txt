cmake_minimum_required(VERSION 3.12)
project(exec_buff)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
    module_behavior
    module_compensator
    spdlog::spdlog
    ${Dcamera}
    ${Dcontroller}
    ${Tbuff}_detector
    ${Tbuff}_object
    ${Tbuff}_predictor
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    $<TARGET_PROPERTY:module_behavior,INTERFACE_INCLUDE_DIRECTORIES>
)

add_compile_options(-g)

add_dependencies(${PROJECT_NAME}
    ${Dcamera}
    ${Dcontroller}
)