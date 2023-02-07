set(CMAKE_INSTALL_EXE_BINDIR ${ROOT_PATH}/build/install/bin/app)
set(CMAKE_INSTALL_DEMO_BINDIR ${ROOT_PATH}/build/install/bin/demo)
set(CMAKE_INSTALL_TEST_BINDIR ${ROOT_PATH}/build/install/bin/test)

function(exe_install target)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        install(
            TARGETS ${target}
            EXPORT ${target}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_EXE_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
            PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        )
        message(STATUS "Installed exe  : ${target}")
    endif()
endfunction()

function(test_install target)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        install(
            TARGETS ${target}
            EXPORT ${target}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_TEST_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
            PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        )
        message(STATUS "Installed test : ${target}")
    endif()
endfunction()

function(demo_install target)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        install(
            TARGETS ${target}
            EXPORT ${target}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_DEMO_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
            PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        )
        message(STATUS "Installed demo : ${target}")
    endif()
endfunction()

function(include_files path)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        file(GLOB INCLUDE_FILE "${path}/*.hpp")
        install(FILES ${INCLUDE_FILE}
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        )
    endif()
endfunction(include_files)

function(lib_install target)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        install(
            TARGETS ${target}
            EXPORT ${target}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )
    endif()
endfunction(lib_install target)
