project(ocra CXX)

include_directories(include
                    ${QUADPROG_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    )

file(GLOB_RECURSE control_srcs src/control/*.cpp)
file(GLOB_RECURSE optim_srcs src/optim/*.cpp)


list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/SFCController.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/ImpedanceController.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/FFIDController.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/DynController.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/Tasks/SFCTask.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/Tasks/ImpedanceTask.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/Tasks/FFIDTask.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/Tasks/DynTask.cpp)


set(OCRA_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE PATH "")

find_package(qpOASES)
if(qpOASES_FOUND)
    include_directories(${qpOASES_INCLUDE_DIRS})
    set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS USE_QPOASES)
else()
    set(qpOASES_LIBRARIES "")
endif()

# Check for SMLT
find_package(SMLT QUIET)
IF(${SMLT_FOUND})
    message(STATUS "-- SMLT lib found. Compiling gaussian process trajectory.")
    include_directories(${SMLT_INCLUDE_DIRS})
ELSE()
    message(STATUS "-- SMLT lib NOT found. Removing gaussian process trajectory from source list.")
    list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/Trajectory/GaussianProcessTrajectory.cpp)
ENDIF()

add_library(ocra-control ${control_srcs})
add_library(ocra-optim ${optim_srcs})

LIST(APPEND link_libs   ${Boost_LIBRARIES}
                        ${QUADPROG_LIBRARIES}
                        ${YARP_LIBRARIES}
                        ${qpOASES_LIBRARIES}
                        )
IF(${SMLT_FOUND})
 LIST(APPEND link_libs ${SMLT_LIBRARIES})
ENDIF()

target_link_libraries(ocra-control ${link_libs})
target_link_libraries(ocra-optim ${link_libs})

install(DIRECTORY include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h*)

install(TARGETS ocra-control ocra-optim
        EXPORT OcraRecipes
        DESTINATION lib)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ocra-control ocra-optim)
set_property(GLOBAL APPEND PROPERTY PACKAGE_${VARS_PREFIX}_INCLUDEDIR include)
