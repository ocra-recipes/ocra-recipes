project(wocra CXX)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${OCRA_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    )
file(GLOB_RECURSE wocra_srcs src/*.cpp)

add_library(wocra ${wocra_srcs})

LIST(APPEND link_libs   ${Boost_LIBRARIES}
                        ${TinyXML_LIBRARIES}
                        ${YARP_LIBRARIES}
                        ocra-control
                        ocra-optim
                        )

set(WOCRA_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE PATH "")

TARGET_LINK_LIBRARIES(wocra ${link_libs})

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h)

install(TARGETS wocra
        EXPORT OcraRecipes
        DESTINATION lib)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS wocra)
# set_property(GLOBAL APPEND PROPERTY PACKAGE_${VARS_PREFIX}_INCLUDEDIR ${CMAKE_INSTALL_PREFIX}/include)
