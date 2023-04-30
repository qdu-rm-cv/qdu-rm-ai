cmake_minimum_required(VERSION 3.12)

# ---------------------------------------------------------------------------------------
# General Components
# ---------------------------------------------------------------------------------------
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/behavior)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/component)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/device)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/vision)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/web)

test_install(test_vision)
test_install(test_device)
test_install(test_component)
test_install(test_behavior)
test_install(test_web)
