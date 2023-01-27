cmake_minimum_required(VERSION 3.12)
project(${Dservo})

# ---------------------------------------------------------------------------------------
# servo
# ---------------------------------------------------------------------------------------
file(GLOB ${Dservo}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/servo.cpp"
)

add_library(${Dservo} STATIC ${${Dservo}_SRC})

target_link_libraries(${Dservo}
    spdlog::spdlog
    module_component
)

target_include_directories(${Dservo} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_dependencies(${Dservo} module_component)