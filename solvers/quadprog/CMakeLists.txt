project(quadprog CXX)

set(QUADPROG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE PATH "")
set(QUADPROG_LIBRARIES quadprog CACHE TARGET "")

find_package(EigenLgsm REQUIRED)

include_directories(
        include
        ${EigenLgsm_INCLUDE_DIRS}
        )

file(GLOB quadprog_srcs src/*.cpp)

add_library(quadprog ${quadprog_srcs})

install(DIRECTORY include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h)

install(TARGETS quadprog
        EXPORT OcraRecipes
        DESTINATION lib)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS quadprog)
set_property(GLOBAL APPEND PROPERTY PACKAGE_${VARS_PREFIX}_INCLUDEDIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
