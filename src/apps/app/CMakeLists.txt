cmake_minimum_required(VERSION 3.12)

# ---------------------------------------------------------------------------------------
# Applications
# ---------------------------------------------------------------------------------------
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/auto_aim)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/buff)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/dart)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sentry)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ui_param)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/aim_assistant)

if(BUILD_NN)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/radar)
endif()