project(ocra CXX)

include_directories(include
                    ${QUADPROG_INCLUDE_DIRS}
                    )
file(GLOB control_srcs src/control/*.cpp)
file(GLOB optim_srcs src/optim/*.cpp)

set(OCRA_CORE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE PATH "")

add_library(ocra-control ${control_srcs})
add_library(ocra-optim ${optim_srcs})

target_link_libraries(ocra-control
    ${Boost_LIBRARIES}
    ${QUADPROG_LIBRARIES}
    )
target_link_libraries(ocra-optim
    ${Boost_LIBRARIES}
    ${QUADPROG_LIBRARIES}
    )

install(DIRECTORY include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h*)

install(TARGETS ocra-control ocra-optim
        EXPORT OcraRecipes
        DESTINATION lib)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ocra-control ocra-optim)
set_property(GLOBAL APPEND PROPERTY PACKAGE_${VARS_PREFIX}_INCLUDEDIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
