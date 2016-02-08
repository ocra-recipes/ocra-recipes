project(gocra CXX)

include_directories(include
                    ${OCRA_CORE_INCLUDE_DIRS}
                    ${QUADPROG_INCLUDE_DIRS}
                    )

file(GLOB_RECURSE gocra_srcs src/*.cpp)

add_library(gocra ${gocra_srcs})

target_link_libraries(gocra ${Boost_LIBRARIES} )

# add_subdirectory(examples)
install(DIRECTORY include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h*)

install(TARGETS gocra
        EXPORT OcraRecipes
        DESTINATION lib)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS gocra)
