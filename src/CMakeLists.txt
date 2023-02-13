cmake_minimum_required(VERSION 3.12)

# ---------------------------------------------------------------------------------------
# General Components
# ---------------------------------------------------------------------------------------
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/behavior)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/component)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/device)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/vision)

# ---------------------------------------------------------------------------------------
# Applictions
# ---------------------------------------------------------------------------------------
if(WITH_UI)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/apps/app)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/apps/demo)
endif()