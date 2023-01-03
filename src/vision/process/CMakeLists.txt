cmake_minimum_required(VERSION 3.12)
project(process)

file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${OpenCV_LIBS}
    classifier
    component
    param
    spdlog::spdlog
    tbb
    ${Taim}_detector
    ${Tbuff}_detector
    ${Tdart}_detector
    ${Tengineer}_detector
    ${Taim}_object
    ${Tbuff}_object
    ${Tdart}_object
    ${Tengineer}_object
    ${Taim}_predictor
    ${Tbuff}_predictor
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)
