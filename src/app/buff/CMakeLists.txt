cmake_minimum_required(VERSION 3.12)
project(buff)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
    device
    object
    detector
    predictor
    process
    compensator
    behavior
    spdlog::spdlog
    ${CERES_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
    $<TARGET_PROPERTY:behavior,INTERFACE_INCLUDE_DIRECTORIES>
)

add_compile_options(-g)

add_dependencies(${PROJECT_NAME}
    device
)