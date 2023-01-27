cmake_minimum_required(VERSION 3.12)
project(demo_camera)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${Dcamera}
    spdlog::spdlog
    ${OpenCV_LIBS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../
    ${CMAKE_CURRENT_SOURCE_DIR}/../../
)

add_dependencies(${PROJECT_NAME}
    ${Dcamera}
)