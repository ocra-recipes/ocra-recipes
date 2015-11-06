cmake_minimum_required(VERSION 2.8)
project(quadprog CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)
message("




CMAKE_CURRENT_SOURCE_DIR : ${CMAKE_CURRENT_SOURCE_DIR}








    ")

set(CMAKE_DEBUG_POSTFIX "_d")

if(MSVC)
    SET(BUILD_SHARED_LIBS OFF)
    set(suffix_type "_s")
    set(install_type "ARCHIVE")
else()
    SET(BUILD_SHARED_LIBS ON)
    set(suffix_type "")
    if(UNIX)
        set(install_type "LIBRARY")
    else()
        set(install_type "RUNTIME")
    endif()
endif()

find_package(Eigen REQUIRED)
find_package(EigenLgsm REQUIRED)

include_directories(
    ${Eigen_INCLUDE_DIRS}
    ${EigenLgsm_INCLUDE_DIR}
)

#set installation
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	set(CMAKE_INSTALL_PREFIX /usr/local/)
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

install(DIRECTORY include/ DESTINATION include/ )


add_subdirectory(pkg-config)
add_subdirectory(include)
add_subdirectory(src)
