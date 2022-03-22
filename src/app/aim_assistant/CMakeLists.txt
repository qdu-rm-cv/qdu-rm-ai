cmake_minimum_required(VERSION 3.12)
project(aim-assistant)

add_executable(${PROJECT_NAME} main.cpp)

if(BUILD_EKF)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        device
        object
        detector
        predictor
        process
        compensator
        behavior 
        spdlog::spdlog 
        ${CERES_LIBRARIES} 
    )

    target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        $<TARGET_PROPERTY:behavior,INTERFACE_INCLUDE_DIRECTORIES>
    )

else()
    target_link_libraries(${PROJECT_NAME} PRIVATE
        device
        object
        detector
        predictor
        process
        compensator
        behavior
        spdlog::spdlog
    )

    target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        $<TARGET_PROPERTY:behavior,INTERFACE_INCLUDE_DIRECTORIES>
    )

endif()

add_dependencies(${PROJECT_NAME}
    device
)