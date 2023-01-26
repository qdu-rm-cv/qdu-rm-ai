cmake_minimum_required(VERSION 3.12)
project(${Dcontroller})

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
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/qdu-rm-protocol/
)

add_dependencies(${Dcontroller} module_component)
