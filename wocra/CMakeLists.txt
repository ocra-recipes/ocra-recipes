project(wocra CXX)

set(lib_name "wocra")


include_directories(include
                    ${OCRA_CORE_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${QUADPROG_INCLUDE_DIRS}
                    )
file(GLOB_RECURSE wocra_srcs src/*.cpp)

# Check for SMLT
find_package(SMLT QUIET)
IF(SMLT_FOUND)
    message("-- SMLT lib found. Compiling gaussian process trajectory.")
    include_directories(SMLT_INCLUDE_DIRS)
ELSE()
    message("-- SMLT lib NOT found. Removing gaussian process trajectory from source list.")
    list(REMOVE_ITEM wocra_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/Trajectory/wOcraGaussianProcessTrajectory.cpp)
ENDIF()


add_library(${lib_name} ${wocra_srcs})



LIST(APPEND link_libs   ${Boost_LIBRARIES}
                        ${TinyXML_LIBRARIES}
                        ${YARP_LIBRARIES}
                        )

IF(SMLT_FOUND)
 LIST(APPEND link_libs ${SMLT_LIBRARIES})
ENDIF()

TARGET_LINK_LIBRARIES(${lib_name} ${link_libs})

install(DIRECTORY include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h)

install(TARGETS ${lib_name}
        EXPORT WOCRATargets
        DESTINATION lib)

# Default CMake package generator
configure_cmake_package(WOCRATargets ${lib_name})
