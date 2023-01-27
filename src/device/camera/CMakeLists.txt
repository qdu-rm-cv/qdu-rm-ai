cmake_minimum_required(VERSION 3.12)
project(${Dcamera})

# ---------------------------------------------------------------------------------------
# camera
# ---------------------------------------------------------------------------------------
file(GLOB ${Dcamera}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/hik_camera.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/raspi_camera.cpp"
)

add_library(${Dcamera} STATIC ${${Dcamera}_SRC})

target_link_libraries(${Dcamera}
    ${OpenCV_LIBS}
    module_component
    MvCameraControl
    spdlog::spdlog
    Threads::Threads
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(${Dcamera} PUBLIC /opt/MVS/lib/aarch64/)
else()
    target_link_directories(${Dcamera} PUBLIC /opt/MVS/lib/64/)
endif()

target_include_directories(${Dcamera} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    /opt/MVS/include
)

add_dependencies(${Dcamera} module_component)
