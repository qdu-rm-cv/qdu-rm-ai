cmake_minimum_required(VERSION 3.12)
project(demo_buff)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
    spdlog::spdlog
    ${Tbuff}_detector
    ${Tbuff}_object
    ${Tbuff}_predictor
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)
