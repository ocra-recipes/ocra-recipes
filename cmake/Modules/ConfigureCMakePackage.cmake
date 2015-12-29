

function(configure_cmake_package export_targets libraries)
    string(TOUPPER ${PROJECT_NAME} CMAKE_PROJECT_NAME)

    message(STATUS "Configuring CMake project ${PROJECT_NAME} to ${CMAKE_PROJECT_NAME}")

    set(extra_libs "${ARGN}")
    list(APPEND libraries ${extra_libs})

    export(PACKAGE ${CMAKE_PROJECT_NAME})

    #cmake_policy(SET CMP0053 NEW)

    include(CMakePackageConfigHelpers)

    set(CMAKE_CONFIG_DEST "share/cmake/${PROJECT_NAME}/")
    set(INCLUDE_INSTALL_DIR "include/")




    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake.in
         "
\@PACKAGE_INIT\@

# - Config file for the \@CMAKE_PROJECT_NAME\@ package
# It defines the following variables
#  \@CMAKE_PROJECT_NAME\@_INCLUDE_DIRS - include directories for \@CMAKE_PROJECT_NAME\@
#  \@CMAKE_PROJECT_NAME\@_LIBRARIES - libs to compile


set_and_check(\@CMAKE_PROJECT_NAME\@_INCLUDE_DIR \"\@PACKAGE_INCLUDE_INSTALL_DIR\@\")
set(\@CMAKE_PROJECT_NAME\@_INCLUDE_DIRS \${\@CMAKE_PROJECT_NAME\@_INCLUDE_DIR})

include(\"\@PACKAGE_CMAKE_CONFIG_DEST\@${export_targets}.cmake\")
set(\@CMAKE_PROJECT_NAME\@_LIBRARIES \"${libraries}\")
")



    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake.in
         "
set(PACKAGE_VERSION \"\@PROJECT_NAME\@\@_VERSION\@\")

# Check whether the requested PACKAGE_FIND_VERSION is compatible
if(\"\${PACKAGE_VERSION}\" VERSION_LESS \"\${PACKAGE_FIND_VERSION}\")
  set(PACKAGE_VERSION_COMPATIBLE FALSE)
else()
  set(PACKAGE_VERSION_COMPATIBLE TRUE)
  if (\"\${PACKAGE_VERSION}\" VERSION_EQUAL \"\${PACKAGE_FIND_VERSION}\")
    set(PACKAGE_VERSION_EXACT TRUE)
  endif()
endif()
")

    configure_package_config_file(
        ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
        INSTALL_DESTINATION ${CMAKE_CONFIG_DEST}
        PATH_VARS INCLUDE_INSTALL_DIR CMAKE_CONFIG_DEST)

    configure_file(${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake" \@ONLY)

    install(FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKE_CONFIG_DEST})

    install(EXPORT ${export_targets} DESTINATION ${CMAKE_CONFIG_DEST})

endfunction()
