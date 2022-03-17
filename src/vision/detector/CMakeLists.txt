cmake_minimum_required(VERSION 3.12)
project(detector)

file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

if(BUILD_NN)
    add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES} )

    target_link_libraries(${PROJECT_NAME} PUBLIC
		${OpenCV_LIBS}
        tbb
		spdlog::spdlog
		component
        object
        param
        classifier
        nvinfer
        nvonnxparser
        cudart
    )

    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        target_include_directories(${PROJECT_NAME} PUBLIC /usr/local/cuda/targets/aarch64-linux/include)
        target_link_directories(${PROJECT_NAME} PUBLIC /usr/local/cuda/targets/aarch64-linux/lib)
    else()
        target_include_directories(${PROJECT_NAME} PUBLIC /usr/local/cuda/include)
        target_link_directories(${PROJECT_NAME} PUBLIC /usr/local/cuda/lib64)
    endif()

else()
    list(FILTER ${PROJECT_NAME}_SOURCES EXCLUDE REGEX "trt.*$")
    list(FILTER ${PROJECT_NAME}_SOURCES EXCLUDE REGEX "radar.*$")

    add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

    target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        tbb
        spdlog::spdlog
        component
        object
        param
        classifier
    )

endif()


target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
