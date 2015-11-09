project(gocra)

set(lib_name "gocra")

include_directories(include
                    ${OCRA_CORE_INCLUDE_DIRS}
                    ${QUADPROG_INCLUDE_DIRS}
                    )

file(GLOB_RECURSE gocra_srcs src/*.cpp)

add_library(${lib_name} ${gocra_srcs})

# target_link_libraries(  ${lib_name}
#                         ocra_optim
#                         ocra_control
#                         # Not sure if these serve any purpose...
#                         # ${Boost_LIBRARIES}
#                         # ${TinyXML_LIBRARIES}
#                         # ${YARP_LIBRARIES}
#                      )


# add_subdirectory(examples)

install(DIRECTORY include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h)

install(TARGETS ${lib_name}
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