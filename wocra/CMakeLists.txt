
set(lib_name "wocra")

include_directories(include
                    ${OCRA_CORE_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    )
file(GLOB_RECURSE wocra_srcs src/*.cpp)

add_library(${lib_name} ${wocra_srcs})

# target_link_libraries(  ${lib_name}
#                         ocra_optim
#                         ocra_control
#                         # Not sure if these serve any purpose...
#                         # ${Boost_LIBRARIES}
#                         # ${TinyXML_LIBRARIES}
#                         # ${YARP_LIBRARIES}
#                      )
