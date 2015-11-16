project(ocra-recipes)
cmake_minimum_required(VERSION 2.8)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

include(ConfigureCMakePackage)

find_package(Eigen REQUIRED)
find_package(EigenLgsm REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(TinyXML REQUIRED)
find_package(YARP REQUIRED)

include_directories(
    ${Eigen_INCLUDE_DIRS}
    ${EigenLgsm_INCLUDE_DIRS}
)

add_subdirectory(ocra)

set(OCRA_CORE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/ocra/include CACHE PATH "")

add_subdirectory(solvers)
add_subdirectory(wocra)
add_subdirectory(gocra)