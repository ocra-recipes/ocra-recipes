project(gocra CXX)

set(lib_name "gocra")

include_directories(include
                    ${OCRA_CORE_INCLUDE_DIRS}
                    ${QUADPROG_INCLUDE_DIRS}
                    )

file(GLOB_RECURSE gocra_srcs src/*.cpp)

add_library(${lib_name} ${gocra_srcs})

target_link_libraries(  ${lib_name}
                        ${Boost_LIBRARIES}
                     )


# add_subdirectory(examples)

install(DIRECTORY include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h)

install(TARGETS ${lib_name}
        EXPORT GOCRATargets
        DESTINATION lib)

# Default CMake package generator
configure_cmake_package(GOCRATargets ${lib_name})
