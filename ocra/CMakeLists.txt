project(ocra CXX)

include_directories(include
                    ${QUADPROG_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    )

file(GLOB control_srcs src/control/*.cpp)
file(GLOB optim_srcs src/optim/*.cpp)

set(OCRA_CORE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE PATH "")

# Check for SMLT
find_package(SMLT QUIET)
IF(${SMLT_FOUND})
    message(STATUS "-- SMLT lib found. Compiling gaussian process trajectory.")
    include_directories(${SMLT_INCLUDE_DIRS})
ELSE()
    message(STATUS "-- SMLT lib NOT found. Removing gaussian process trajectory from source list.")
    list(REMOVE_ITEM control_srcs src/Trajectory/GaussianProcessTrajectory.cpp)
ENDIF()

add_library(ocra-control ${control_srcs})
add_library(ocra-optim ${optim_srcs})

LIST(APPEND link_libs   ${Boost_LIBRARIES}
                        ${QUADPROG_LIBRARIES}
                        ${YARP_LIBRARIES}
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
set_property(GLOBAL APPEND PROPERTY PACKAGE_${VARS_PREFIX}_INCLUDEDIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
