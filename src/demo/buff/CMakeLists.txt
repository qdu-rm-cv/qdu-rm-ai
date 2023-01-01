cmake_minimum_required(VERSION 3.12)
project(buff_demo)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
    device
    object
    detector
    predictor
    spdlog::spdlog
    ${CERES_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)

add_dependencies(${PROJECT_NAME}
    device
)