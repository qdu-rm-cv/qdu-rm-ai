cmake_minimum_required(VERSION 3.12)
project(sentry)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
    behavior
    compensator
    device
    spdlog::spdlog
    ${Taim}_detector
    ${Taim}_object
    ${Taim}_predictor
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(${PROJECT_NAME}
    behavior
    device
)
