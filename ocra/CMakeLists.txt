project(ocra)

include_directories(include)
file(GLOB control_srcs src/control/*.cpp)
file(GLOB optim_srcs src/optim/*.cpp)


add_library(ocra_control ${control_srcs})
add_library(ocra_optim ${optim_srcs})

# target_link_libraries(ocra_control
#     ${Boost_LIBRARIES}
#     )
# target_link_libraries(ocra_optim
#     ${Boost_LIBRARIES}
#     )

install(DIRECTORY include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h)

install(TARGETS ocra_control ocra_optim
        DESTINATION lib)


##########################################################################
###### Cmake package config ##############################################

include(CMakePackageConfigHelpers)

set(CMAKE_CONFIG_DEST "share/cmake/${PROJECT_NAME}/")
set(INCLUDE_INSTALL_DIR "include/${PROJECT_NAME}")
string(TOUPPER ${PROJECT_NAME} CMAKE_PROJECT_NAME)

configure_package_config_file(
       ${CMAKE_CURRENT_SOURCE_DIR}/${CMAKE_PROJECT_NAME}Config.cmake.in
       ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
       INSTALL_DESTINATION ${CMAKE_CONFIG_DEST}
       PATH_VARS INCLUDE_INSTALL_DIR CMAKE_CONFIG_DEST)

#configure_file(${CMAKE_PROJECT_NAME}ConfigVersion.cmake.in
# "${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake" @ONLY)

install(FILES
 "${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake"
# "${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake"
 DESTINATION ${CMAKE_CONFIG_DEST})
