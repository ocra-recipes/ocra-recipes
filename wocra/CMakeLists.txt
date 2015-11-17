project(wocra CXX)

LIST(APPEND CMAKE_CXX_FLAGS "-fPIC")

set(lib_name "wocra")

include_directories(include
                    ${OCRA_CORE_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    )
file(GLOB_RECURSE wocra_srcs src/*.cpp)

add_library(${lib_name} ${wocra_srcs})
# 
# target_link_libraries(  ${lib_name}
#                         ocra_optim
#                         ocra_control
#                         ${Boost_LIBRARIES}
#                         ${TinyXML_LIBRARIES}
#                         ${YARP_LIBRARIES}
#                      )

install(DIRECTORY include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN *.h)

install(TARGETS ${lib_name}
        EXPORT WOCRATargets
        DESTINATION lib)

# Default CMake package generator
configure_cmake_package(WOCRATargets ${lib_name})
