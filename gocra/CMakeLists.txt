
set(lib_name "gocra")

include_directories(include
                    ${OCRA_CORE_INCLUDE_DIRS}
                    ${QUADPROG_INCLUDE_DIRS}
                    )

file(GLOB_RECURSE gocra_srcs src/*.cpp)

add_library(${lib_name} ${gocra_srcs})

# target_link_libraries(  ${lib_name}
#                         ocra_optim
#                         ocra_control
#                         # Not sure if these serve any purpose...
#                         # ${Boost_LIBRARIES}
#                         # ${TinyXML_LIBRARIES}
#                         # ${YARP_LIBRARIES}
#                      )


# add_subdirectory(examples)
