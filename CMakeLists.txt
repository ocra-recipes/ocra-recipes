cmake_minimum_required(VERSION 2.8.11)

project(OcraRecipes)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

if(APPLE)
    set(BUILD_SHARED_LIBS FALSE)
else()
    set(BUILD_SHARED_LIBS TRUE)
endif()


option(USING_ATOM_EDITOR "If using atom as an IDE build the necessary json files." FALSE)
if(USING_ATOM_EDITOR)
    # https://atom.io/packages/linter-clang
    # http://clang.llvm.org/docs/JSONCompilationDatabase.html
    message("-- Using Atom editor as a C++ IDE. Generating necessary files.")
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

option(BUILD_TESTS "(Developers Only) Build the various unit tests." FALSE)


set(VARS_PREFIX "OcraRecipes")

set(${VARS_PREFIX}_MAJOR_VERSION 1)
set(${VARS_PREFIX}_MINOR_VERSION 0)
set(${VARS_PREFIX}_PATCH_VERSION 0)
set(${VARS_PREFIX}_VERSION ${${VARS_PREFIX}_MAJOR_VERSION}.${${VARS_PREFIX}_MINOR_VERSION}.${${VARS_PREFIX}_PATCH_VERSION})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

#set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)


set(CMAKE_POSITION_INDEPENDENT_CODE "TRUE")

find_package(Eigen REQUIRED)
find_package(EigenLgsm REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(TinyXML REQUIRED)
find_package(YARP REQUIRED)
find_package(qpOASES REQUIRED)

include_directories( AFTER
    ${Eigen_INCLUDE_DIRS}
    ${EigenLgsm_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
    ${qpOASES_INCLUDE_DIRS}
)

add_subdirectory(ocra)
add_subdirectory(wocra)
add_subdirectory(hocra)
add_subdirectory(ocra-recipes)
# add_subdirectory(gocra)

set_property(GLOBAL APPEND PROPERTY PACKAGE_${VARS_PREFIX}_INCLUDEDIR ${CMAKE_INSTALL_PREFIX}/include)

# Build tests if option is activated.
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

# add a target to generate API documentation with Doxygen
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in
                   ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/latexcustomcommands.sty
                   ${CMAKE_CURRENT_BINARY_DIR}/docs/html/latexcustomcommands.sty COPYONLY)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/latexcustomcommands.sty
                   ${CMAKE_CURRENT_SOURCE_DIR}/../../main/ocra-wbi-plugins/html-gh-pages/doxy-ocra-recipes/html/latexcustomcommands.sty COPYONLY)
    message("-- Copied file ocraReferences.bib into html-gh-pages " ${CMAKE_CURRENT_BINARY_DIR}/../../main/ocra-wbi-plugins/html-gh-pages/doxy-ocra-recipes/html/ocraReferences.bib)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/ocraReferences.bib
                   ${CMAKE_CURRENT_BINARY_DIR}/../../libraries/ocra-recipes/docs/ocraReferences.bib COPYONLY)
    add_custom_target(docs
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif()

include(InstallBasicPackageFiles)
get_property(PACKAGE_${VARS_PREFIX}_INCLUDEDIR GLOBAL PROPERTY PACKAGE_${VARS_PREFIX}_INCLUDEDIR)
install_basic_package_files(OcraRecipes VARS_PREFIX ${VARS_PREFIX}
                                        VERSION ${${VARS_PREFIX}_VERSION}
                                        COMPATIBILITY SameMajorVersion
                                        TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                        NO_CHECK_REQUIRED_COMPONENTS_MACRO
                                    )
include(AddUninstallTarget)


if(USING_ATOM_EDITOR)
    add_custom_target(  atom-files ALL
                        DEPENDS ocra-recipes
                        COMMAND ${CMAKE_COMMAND} -E copy
                        ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
                        ${CMAKE_SOURCE_DIR}
                        COMMENT "Copying 'compile_commands.json' to '${CMAKE_SOURCE_DIR}'"
                     )
endif()
