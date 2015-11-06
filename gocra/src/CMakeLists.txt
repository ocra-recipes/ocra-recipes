
include_directories(
	${PROJECT_SOURCE_DIR}/include
    ${PC_OCRA_FRAMEWORK_INCLUDE_DIRS}
    ${PC_QUADPROG_INCLUDE_DIRS}
    ${PC_EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)



file(
	GLOB_RECURSE
	source_files

	*.cpp
)


LINK_DIRECTORIES(
    ${PC_OCRA_FRAMEWORK_LIBRARY_DIRS}
    ${PC_QUADPROG_LIBRARY_DIRS}
)


add_library(
	${PROJECTNAME} ${suffix_type}
	${source_files}
)

MESSAGE(STATUS "ocra libs:	" ${PC_OCRA_FRAMEWORK_LIBRARY_DIRS})

target_link_libraries(
    ${PROJECTNAME}
    ${PC_QUADPROG_LIBRARIES}
    ${PC_OCRA_FRAMEWORK_LIBRARIES}
)



install(
    TARGETS ${PROJECTNAME} DESTINATION lib
)
