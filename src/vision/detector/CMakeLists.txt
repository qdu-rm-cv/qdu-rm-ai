cmake_minimum_required(VERSION 3.12)
project(detector)

file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

if(BUILD_NN)
    # ---------------------------------------------------------------------------------------
    # radar
    # ---------------------------------------------------------------------------------------
    file(GLOB ${Tradar}_${PROJECT_NAME}_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/radar_detector.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/trt_detector.cpp"
    )

    add_library(${Tradar}_${PROJECT_NAME} STATIC ${${Tradar}_${PROJECT_NAME}_SRC})

    target_link_libraries(${Tradar}_${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        component
        cudart
        nvinfer
        nvonnxparser
        param
        spdlog::spdlog
        tbb
        ${Taim}_object
    )

    target_include_directories(${Tradar}_${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        target_include_directories(${Tradar}_${PROJECT_NAME} PUBLIC /usr/local/cuda/targets/aarch64-linux/include)
        target_link_directories(${Tradar}_${PROJECT_NAME} PUBLIC /usr/local/cuda/targets/aarch64-linux/lib)
    else()
        target_include_directories(${Tradar}_${PROJECT_NAME} PUBLIC /usr/local/cuda/include)
        target_link_directories(${Tradar}_${PROJECT_NAME} PUBLIC /usr/local/cuda/lib64)
    endif()
endif()

# ---------------------------------------------------------------------------------------
# aim
# ---------------------------------------------------------------------------------------
file(GLOB ${Taim}_${PROJECT_NAME}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/armor_detector.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/async_armor_detector.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/snipe_detector.cpp"
)

add_library(${Taim}_${PROJECT_NAME} STATIC ${${Taim}_${PROJECT_NAME}_SRC})

target_link_libraries(${Taim}_${PROJECT_NAME} PUBLIC
    ${OpenCV_LIBS}
    component
    param
    spdlog::spdlog
    tbb
    ${Taim}_object
)

target_include_directories(${Taim}_${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ---------------------------------------------------------------------------------------
# buff
# ---------------------------------------------------------------------------------------
file(GLOB ${Tbuff}_${PROJECT_NAME}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/buff_detector.cpp"
)

add_library(${Tbuff}_${PROJECT_NAME} STATIC ${${Tbuff}_${PROJECT_NAME}_SRC})

target_link_libraries(${Tbuff}_${PROJECT_NAME} PUBLIC
    ${OpenCV_LIBS}
    component
    param
    spdlog::spdlog
    tbb
    ${Tbuff}_object
)

target_include_directories(${Tbuff}_${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ---------------------------------------------------------------------------------------
# dart
# ---------------------------------------------------------------------------------------
file(GLOB ${Tdart}_${PROJECT_NAME}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/guidinglight_detector.cpp"
)

add_library(${Tdart}_${PROJECT_NAME} STATIC ${${Tdart}_${PROJECT_NAME}_SRC})

target_link_libraries(${Tdart}_${PROJECT_NAME} PUBLIC
    ${OpenCV_LIBS}
    component
    param
    spdlog::spdlog
    tbb
    ${Tdart}_object
)

target_include_directories(${Tdart}_${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ---------------------------------------------------------------------------------------
# engineer
# ---------------------------------------------------------------------------------------
file(GLOB ${Tengineer}_${PROJECT_NAME}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/orecube_detector.cpp"
)

add_library(${Tengineer}_${PROJECT_NAME} STATIC ${${Tengineer}_${PROJECT_NAME}_SRC})

target_link_libraries(${Tengineer}_${PROJECT_NAME} PUBLIC
    ${OpenCV_LIBS}
    component
    param
    spdlog::spdlog
    tbb
    ${Tengineer}_object
)

target_include_directories(${Tengineer}_${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
