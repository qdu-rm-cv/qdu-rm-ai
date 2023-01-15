cmake_minimum_required(VERSION 3.12)
project(object)

# ---------------------------------------------------------------------------------------
# object_base
# ---------------------------------------------------------------------------------------
file(GLOB module_${PROJECT_NAME}_base_SRC "${CMAKE_CURRENT_SOURCE_DIR}/object.cpp")

add_library(module_${PROJECT_NAME}_base STATIC ${module_${PROJECT_NAME}_base_SRC})

target_link_libraries(module_${PROJECT_NAME}_base PUBLIC
    ${OpenCV_LIBS}
    tbb
    spdlog::spdlog
    module_component
)

target_include_directories(module_${PROJECT_NAME}_base PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ---------------------------------------------------------------------------------------
# aim
# ---------------------------------------------------------------------------------------
file(GLOB ${Taim}_${PROJECT_NAME}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/light_bar.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/armor.cpp"
)

add_library(${Taim}_${PROJECT_NAME} STATIC ${${Taim}_${PROJECT_NAME}_SRC})

target_link_libraries(${Taim}_${PROJECT_NAME} PUBLIC
    module_${PROJECT_NAME}_base
    ${OpenCV_LIBS}
    tbb
    spdlog::spdlog
    module_component
)

target_include_directories(${Taim}_${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ---------------------------------------------------------------------------------------
# buff
# ---------------------------------------------------------------------------------------
file(GLOB ${Tbuff}_${PROJECT_NAME}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/buff.cpp"
)

add_library(${Tbuff}_${PROJECT_NAME} STATIC ${${Tbuff}_${PROJECT_NAME}_SRC})

target_link_libraries(${Tbuff}_${PROJECT_NAME} PUBLIC
    ${Taim}_${PROJECT_NAME}
    ${OpenCV_LIBS}
    tbb
    spdlog::spdlog
    module_component
)

target_include_directories(${Tbuff}_${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ---------------------------------------------------------------------------------------
# dart
# ---------------------------------------------------------------------------------------
file(GLOB ${Tdart}_${PROJECT_NAME}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/guiding_light.cpp"
)

add_library(${Tdart}_${PROJECT_NAME} STATIC ${${Tdart}_${PROJECT_NAME}_SRC})

target_link_libraries(${Tdart}_${PROJECT_NAME} PUBLIC
    module_${PROJECT_NAME}_base
    ${OpenCV_LIBS}
    tbb
    spdlog::spdlog
    module_component
)

target_include_directories(${Tdart}_${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ---------------------------------------------------------------------------------------
# engineer
# ---------------------------------------------------------------------------------------
file(GLOB ${Tengineer}_${PROJECT_NAME}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/ore_cube.cpp"
)

add_library(${Tengineer}_${PROJECT_NAME} STATIC ${${Tengineer}_${PROJECT_NAME}_SRC})

target_link_libraries(${Tengineer}_${PROJECT_NAME} PUBLIC
    module_${PROJECT_NAME}_base
    ${OpenCV_LIBS}
    tbb
    spdlog::spdlog
    module_component
)
target_include_directories(${Tengineer}_${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
