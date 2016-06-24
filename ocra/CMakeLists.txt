project(ocra CXX)

include_directories(BEFORE ${PROJECT_SOURCE_DIR}/include)

file(GLOB_RECURSE control_srcs src/control/*.cpp)
file(GLOB_RECURSE optim_srcs src/optim/*.cpp)
file(GLOB_RECURSE util_srcs src/util/*.cpp)




set(OCRA_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE PATH "")

# Check for SMLT
find_package(SMLT QUIET)
IF(${SMLT_FOUND})
    message(STATUS "SMLT lib found. Compiling gaussian process trajectory.")
    include_directories(${SMLT_INCLUDE_DIRS})
ELSE()
    message(STATUS "SMLT lib NOT found. Removing gaussian process trajectory from source list.")
    list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/Trajectory/GaussianProcessTrajectory.cpp)
ENDIF()

list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/TaskManagers/TaskManager.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/TaskManagers/CoMTaskManager.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/TaskManagers/CoMMomentumTaskManager.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/TaskManagers/ContactTaskManager.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/TaskManagers/ContactSetTaskManager.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/TaskManagers/FullPostureTaskManager.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/TaskManagers/PartialPostureTaskManager.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/TaskManagers/SegCartesianTaskManager.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/TaskManagers/SegOrientationTaskManager.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/TaskManagers/SegPoseTaskManager.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/TaskManagers/TaskManagerSet.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/TaskManagers/TaskSequence.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/TaskManagers/TaskManagerOptions.cpp)
list(REMOVE_ITEM control_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/control/TaskManagers/TaskManagerFactory.cpp)



add_library(ocra-control ${control_srcs})
add_library(ocra-optim ${optim_srcs})
add_library(ocra-util ${util_srcs})

LIST(APPEND link_libs   ${Boost_LIBRARIES}
                        ${YARP_LIBRARIES}
                        ${qpOASES_LIBRARIES}
                        )
IF(${SMLT_FOUND})
 LIST(APPEND link_libs ${SMLT_LIBRARIES})
ENDIF()

target_link_libraries(ocra-control ${link_libs})
target_link_libraries(ocra-optim ${link_libs})
target_link_libraries(ocra-util ${link_libs})

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h*)

install(TARGETS ocra-control ocra-optim ocra-util
        EXPORT OcraRecipes
        DESTINATION lib)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ocra-control ocra-optim ocra-util)
# set_property(GLOBAL APPEND PROPERTY PACKAGE_${VARS_PREFIX}_INCLUDEDIR ${CMAKE_INSTALL_PREFIX}/include)
