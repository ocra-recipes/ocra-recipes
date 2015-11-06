cmake_minimum_required(VERSION 2.8)
project(wLQP-Control CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_DEBUG_POSTFIX "_d")
set(CMAKE_CXX_FLAGS "-fpermissive")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

if(MSVC)
    SET(BUILD_SHARED_LIBS OFF)
    set(suffix_type "_static")
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


# Check dependencies
find_package(PkgConfig REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(YARP REQUIRED)
find_package(TinyXML QUIET)
IF(NOT TinyXML_FOUND)
    message("Couldn't find TinyXML using cmake pakages. Trying with pkg-config...")
    PKG_CHECK_MODULES(TinyXML tinyxml REQUIRED)
ENDIF()


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
