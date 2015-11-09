
set(lib_name "ocra-core")

include_directories(include)
file(GLOB control_srcs src/control/*.cpp)
file(GLOB optim_srcs src/optim/*.cpp)


add_library(ocra_control ${control_srcs})
add_library(ocra_optim ${optim_srcs})

# target_link_libraries(ocra_control
#     ${Boost_LIBRARIES}
#     )
# target_link_libraries(ocra_optim
#     ${Boost_LIBRARIES}
#     )
