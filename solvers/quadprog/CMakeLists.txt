project(quadprog CXX)

set(lib_name "quadprog")

set(QUADPROG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE PATH "")


include_directories(include)
file(GLOB quadprog_srcs src/*.cpp)

add_library(quadprog ${quadprog_srcs})

install(DIRECTORY include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h)

install(TARGETS ${lib_name}
        EXPORT QUADPROGTargets
        DESTINATION lib)

# Default CMake package generator
configure_cmake_package(QUADPROGTargets ${lib_name})
