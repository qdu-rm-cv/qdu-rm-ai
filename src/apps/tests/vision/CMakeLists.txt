cmake_minimum_required(VERSION 3.12)
project(test_vision)

file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

if(BUILD_NN)
    add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})

    target_link_libraries(${PROJECT_NAME} PRIVATE
        module_classifier
        module_compensator
        cudart
        gtest
        gtest_main
        nvinfer
        nvonnxparser
        ${Taim}_detector
        ${Tbuff}_detector
        ${Tdart}_detector
        ${Tengineer}_detector
        ${Tradar}_detector
        ${Taim}_object
        ${Tbuff}_object
        ${Tdart}_object
        ${Tengineer}_object
        ${Tbuff}_predictor
    )

    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        target_include_directories(${PROJECT_NAME} PUBLIC /usr/local/cuda/targets/aarch64-linux/include)
        target_link_directories(${PROJECT_NAME} PUBLIC /usr/local/cuda/targets/aarch64-linux/lib)
    else()
        target_include_directories(${PROJECT_NAME} PUBLIC /usr/local/cuda/include)
        target_link_directories(${PROJECT_NAME} PUBLIC /usr/local/cuda/lib64)
    endif()

    target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        $<TARGET_PROPERTY:module_classifier,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:module_compensator,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Taim}_detector,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tbuff}_detector,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tdart}_detector,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tengineer}_detector,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tradar}_detector,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tbuff}_object,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tdart}_object,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tengineer}_object,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Taim}_predictor,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tbuff}_predictor,INTERFACE_INCLUDE_DIRECTORIES>
    )

elseif(NOT BUILD_NN)
    list(FILTER ${PROJECT_NAME}_SOURCES EXCLUDE REGEX "trt.*$")
    list(FILTER ${PROJECT_NAME}_SOURCES EXCLUDE REGEX "radar.*$")

    add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})

    target_link_libraries(${PROJECT_NAME} PRIVATE
        module_classifier
        module_compensator
        gtest
        gtest_main
        ${Taim}_detector
        ${Tbuff}_detector
        ${Tdart}_detector
        ${Tengineer}_detector
        ${Taim}_object
        ${Tbuff}_object
        ${Tdart}_object
        ${Tengineer}_object
        ${Tbuff}_predictor
    )

    target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        $<TARGET_PROPERTY:module_classifier,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:module_compensator,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Taim}_detector,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tbuff}_detector,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tdart}_object,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tengineer}_detector,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Taim}_object,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tbuff}_object,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tdart}_object,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tengineer}_object,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:${Tbuff}_predictor,INTERFACE_INCLUDE_DIRECTORIES>
    )
endif()

add_test(test_vision test_vision)
