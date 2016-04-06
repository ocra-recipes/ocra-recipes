project(ocra-recipes)

include_directories(BEFORE include)
include_directories(
                    ${QUADPROG_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${OCRA_INCLUDE_DIRS}
                    ${WOCRA_INCLUDE_DIRS}
                    )

file(GLOB_RECURSE source_files src/*.cpp)

add_library(${PROJECT_NAME} ${source_files})

LIST(APPEND link_libs   ${Boost_LIBRARIES}
                        ${QUADPROG_LIBRARIES}
                        ${YARP_LIBRARIES}
                        ocra-control
                        ocra-optim
                        wocra
                        )

target_link_libraries(${PROJECT_NAME} ${link_libs})

install(DIRECTORY include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h*)

install(TARGETS ${PROJECT_NAME}
        EXPORT OcraRecipes
        DESTINATION lib)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${PROJECT_NAME})
set_property(GLOBAL APPEND PROPERTY PACKAGE_${VARS_PREFIX}_INCLUDEDIR include)
