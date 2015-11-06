include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
    ${ocra_framework_SOURCE_DIR}/include
    ${quadprog_SOURCE_DIR}/include
    ${PC_EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${YARP_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
    ${ocra_framework_SOURCE_DIR}/src
    ${quadprog_SOURCE_DIR}/src
)

file(
	GLOB_RECURSE
	source_files

	*.cpp
)

add_library(
	wLQP-Control${suffix_type}
	${LIB_TYPE}
	${source_files}
)

target_link_libraries(
    wLQP-Control
    ocra_control
    ocra_optim
    quadprog
    ${YARP_LIBRARIES}
    ${TinyXML_LIBRARIES}
    ${Boost_LIBRARIES}
)

install(
    TARGETS wLQP-Control
    ${install_type} DESTINATION lib
)
