cmake_minimum_required(VERSION 3.12)
project(predictor)

# ---------------------------------------------------------------------------------------
# predictor_base
# ---------------------------------------------------------------------------------------
file(GLOB module_${PROJECT_NAME}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/ekf.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/kalman.cpp"
)

add_library(module_${PROJECT_NAME}_base STATIC ${module_${PROJECT_NAME}_SRC})

target_link_libraries(module_${PROJECT_NAME}_base PUBLIC
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
    module_component
    spdlog::spdlog
    tbb
)

target_include_directories(module_${PROJECT_NAME}_base PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------------------
# aim
# ---------------------------------------------------------------------------------------
file(GLOB ${Taim}_${PROJECT_NAME}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/armor_predictor.cpp"
)

add_library(${Taim}_${PROJECT_NAME} STATIC ${${Taim}_${PROJECT_NAME}_SRC})

target_link_libraries(${Taim}_${PROJECT_NAME} PUBLIC
    module_${PROJECT_NAME}_base
    ${OpenCV_LIBS}
    module_component
    spdlog::spdlog
    tbb
    ${Taim}_detector
    ${Taim}_object
)

target_include_directories(${Taim}_${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ---------------------------------------------------------------------------------------
# buff
# ---------------------------------------------------------------------------------------
file(GLOB ${Tbuff}_${PROJECT_NAME}_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/buff_predictor.cpp"
)

add_library(${Tbuff}_${PROJECT_NAME} STATIC ${${Tbuff}_${PROJECT_NAME}_SRC})

target_link_libraries(${Tbuff}_${PROJECT_NAME} PUBLIC
    module_${PROJECT_NAME}_base
    ${OpenCV_LIBS}
    module_component
    spdlog::spdlog
    tbb
    ${Tbuff}_detector
    ${Tbuff}_object
)

target_include_directories(${Tbuff}_${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
