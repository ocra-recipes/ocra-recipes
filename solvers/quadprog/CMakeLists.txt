
set(lib_name "quadprog")

include_directories(include)
file(GLOB quadprog_srcs src/*.cpp)

message("Files:

${quadprog_srcs}

")

add_library(quadprog ${quadprog_srcs})
