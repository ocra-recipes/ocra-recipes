
cmake_minimum_required(VERSION 2.8)

project(test_3T CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-fpermissive")


find_package(PkgConfig REQUIRED)

PKG_CHECK_MODULES(PC_EIGEN eigen3 REQUIRED)
if(NOT PC_EIGEN_FOUND)
    message("Failed to detect Eigen, check PKG_CONFIG_PATH." )
endif(NOT PC_EIGEN_FOUND)

PKG_CHECK_MODULES(PC_GOCRA gOcra REQUIRED)
if(NOT PC_GOCRA_FOUND)
    message("Failed to detect gOcra, check PKG_CONFIG_PATH." )
endif(NOT PC_GOCRA_FOUND)



include_directories(
    ${PC_GOCRA_INCLUDE_DIRS}
    ${PC_EIGEN_INCLUDE_DIRS}
)
LINK_DIRECTORIES(
    ${PC_GOCRA_LIBRARY_DIRS}
)

#========================================
add_library(
    Model3T
    SHARED
    Model3T.cpp
)

target_link_libraries(
    Model3T
    ${PC_GOCRA_LIBRARIES}
)

#========================================
add_executable(
    test_3T
    main.cpp
)

target_link_libraries(
    test_3T
    ${PC_GOCRA_LIBRARIES}
    Model3T
)

