cmake_minimum_required(VERSION 3.12)
project(device)

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

# ---------------------------------------------------------------------------------------
# controller
# ---------------------------------------------------------------------------------------
file(GLOB ${Dcontroller}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/crc16.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/serial.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/robot.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/usb.cpp"
)

add_library(${Dcontroller} STATIC ${${Dcontroller}_SRC})

target_link_libraries(${Dcontroller}
    ${OpenCV_LIBS}
    module_component
    spdlog::spdlog
    Threads::Threads
    usbp-1
)

target_include_directories(${Dcontroller} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/qdu-rm-protocol/
)

add_dependencies(${Dcontroller} module_component)

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