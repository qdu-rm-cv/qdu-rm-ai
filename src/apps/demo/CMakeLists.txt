cmake_minimum_required(VERSION 3.12)

# ---------------------------------------------------------------------------------------
# Demo
# ---------------------------------------------------------------------------------------
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/armor)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/buff)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/camera)

demo_install(demo_armor)
demo_install(demo_buff)
demo_install(demo_camera)