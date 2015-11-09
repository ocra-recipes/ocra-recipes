
set(lib_name "quadprog")

set(QUADPROG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE PATH "")


include_directories(include)
file(GLOB quadprog_srcs src/*.cpp)

add_library(quadprog ${quadprog_srcs})
