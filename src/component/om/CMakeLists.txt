cmake_minimum_required(VERSION 3.11)
project(OM)

add_subdirectory(${SUBMODULE_PATH}/OneMessageCPP ${CMAKE_BINARY_DIR}/om.out)

configure_file(om_config.h.in ${CMAKE_BINARY_DIR}/generated/om_config.h)

target_include_directories(OneMessage PUBLIC
    ${CMAKE_BINARY_DIR}/generated/
)

set_target_properties(OneMessage PROPERTIES OUTPUT_NAME "OM")
