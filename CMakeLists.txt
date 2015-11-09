project(ocra-recipes)
cmake_minimum_required(VERSION 2.8)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

#list(APPEND CMAKE_CXX_FLAGS "-fpermissive")

find_package(Eigen REQUIRED)
find_package(EigenLgsm REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(TinyXML REQUIRED)
find_package(YARP REQUIRED)

include_directories(
    ${Eigen_INCLUDE_DIRS}
    ${EigenLgsm_INCLUDE_DIRS}
    # ${Boost_INCLUDE_DIRS}
    # ${TinyXML_INCLUDE_DIRS}
)

add_subdirectory(ocra)

set(OCRA_CORE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/ocra/include CACHE PATH "")

add_subdirectory(solvers)
add_subdirectory(wocra)
add_subdirectory(gocra)

function(configure_cmake_package)
    ##########################################################################
    ###### Cmake package config ##############################################

    include(CMakePackageConfigHelpers)

    set(CMAKE_CONFIG_DEST "share/cmake/${PROJECT_NAME}/")
    set(INCLUDE_INSTALL_DIR "include/${PROJECT_NAME}")
    string(TOUPPER PROJECT_NAME CMAKE_PROJECT_NAME)

    configure_package_config_file(                         
        ${CMAKE_SOURCE_DIR}/${CMAKE_PROJECT_NAME}Config.cmake.in
        ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
        INSTALL_DESTINATION ${CMAKE_CONFIG_DEST}
        PATH_VARS INCLUDE_INSTALL_DIR CMAKE_CONFIG_DEST)

    configure_file(${CMAKE_PROJECT_NAME}ConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake" @ONLY)

    install(FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKE_CONFIG_DEST})
endfunction()