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

exe_install(exec_auto_aim)
exe_install(exec_buff)
exe_install(exec_dart)
exe_install(exec_sentry)
exe_install(exec_ui_param)

exe_install(exec_aim_assistant)

if(BUILD_NN)
    exe_install(exec_radar)
endif()