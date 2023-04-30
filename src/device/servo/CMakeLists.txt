cmake_minimum_required(VERSION 3.12)
project(${Dservo})

# ---------------------------------------------------------------------------------------
# servo
# ---------------------------------------------------------------------------------------
file(GLOB ${PROJECT_NAME}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/servo.cpp"
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SRC})

target_link_libraries(${PROJECT_NAME}
    spdlog::spdlog
    module_component
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_dependencies(${PROJECT_NAME} module_component)