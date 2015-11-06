cmake_minimum_required(VERSION 2.8)
project(ocra_framework CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-fpermissive")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

if(MSVC)
    set(LIB_TYPE "STATIC")
    set(suffix_type "_static")
    set(install_type "ARCHIVE")
else()
    set(LIB_TYPE "SHARED")
    set(suffix_type "")
    if (UNIX)
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


include_directories(include)
include_directories(include/ocra/control)
include_directories(include/ocra/optim)
include_directories(${PC_EIGEN_INCLUDE_DIRS})



add_subdirectory(src)
add_subdirectory(pkg-config)



#set installation
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	set(CMAKE_INSTALL_PREFIX /usr/local/)
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

install(DIRECTORY include/ DESTINATION include/ )
install(
    DIRECTORY cmake/Modules/
    DESTINATION share/cmake/Modules/
)



message("${LIBDIR_KW}")
