cmake_minimum_required(VERSION 3.12)
project(param)

file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# ---------------------------------------------------------------------------------------
# param_base
# ---------------------------------------------------------------------------------------
# file(GLOB ${PROJECT_NAME}_BASE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/object.cpp")

# add_library(${PROJECT_NAME}_BASE STATIC ${${PROJECT_NAME}_BASE_SRC})
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OpenCV_LIBS}
    tbb
    spdlog::spdlog
    component
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)