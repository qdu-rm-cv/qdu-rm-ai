cmake_minimum_required(VERSION 3.12)
project(predictor)

file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

if(BUILD_EKF)
    add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

    target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        tbb
        spdlog::spdlog
        object
        detector
        component
        ${CERES_LIBRARIES} 
    )
    
    target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
    )

else()
    list(FILTER ${PROJECT_NAME}_SOURCES EXCLUDE REGEX "ekf.*$")

    add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

    target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        tbb
        spdlog::spdlog
        object
        detector
        component
    )

    target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

endif()