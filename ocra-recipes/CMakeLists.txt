set(PROJECTNAME ocra-recipes)
project(${PROJECTNAME})

include_directories(BEFORE ${PROJECT_SOURCE_DIR}/include)
include_directories(BEFORE ${OCRA_INCLUDE_DIRS})
include_directories(
                    ${YARP_INCLUDE_DIRS}
                    ${WOCRA_INCLUDE_DIRS}
                    ${HOCRA_INCLUDE_DIRS}
                    )

find_package(SMLT QUIET)
if(${SMLT_FOUND})
    include_directories(${SMLT_INCLUDE_DIRS})
    add_definitions(-DUSING_SMLT=1)
endif()
find_package(GTTraj QUIET)
if(${GTTraj_FOUND})
    include_directories(${GTTraj_INCLUDE_DIRS})
    add_definitions(-DUSING_GTTRAJ=1)
endif()



file(GLOB_RECURSE source_files src/*.cpp)
file(GLOB_RECURSE header_files include/${PROJECTNAME}/*.h )

SOURCE_GROUP("Source Files" FILES ${source_files})
SOURCE_GROUP("Header Files" FILES ${header_files})

add_library(${PROJECT_NAME} ${source_files} ${header_files})

LIST(APPEND link_libs   ${Boost_LIBRARIES}
                        ${YARP_LIBRARIES}
                        ocra-control
                        ocra-optim
                        wocra
                        hocra
                        )

target_link_libraries(${PROJECT_NAME} ${link_libs})

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h*)

install(TARGETS ${PROJECT_NAME}
        EXPORT OcraRecipes
        DESTINATION lib)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${PROJECT_NAME})
# set_property(GLOBAL APPEND PROPERTY PACKAGE_${VARS_PREFIX}_INCLUDEDIR ${CMAKE_INSTALL_PREFIX}/include)
