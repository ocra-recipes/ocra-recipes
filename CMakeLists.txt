project(ocra)
cmake_minimum_required(VERSION 2.8)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

list(APPEND CMAKE_CXX_FLAGS "-fpermissive")

find_package(Eigen REQUIRED)
find_package(EigenLgsm REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(TinyXML REQUIRED)

include_directories(
    ${Eigen_INCLUDE_DIRS}
    ${EigenLgsm_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
)

message("


${EigenLgsm_INCLUDE_DIRS}


")
add_subdirectory(ocra-core)
#add_subdirectory(wocra)
#add_subdirectory(gocra)
#add_subdirectory(solvers)
