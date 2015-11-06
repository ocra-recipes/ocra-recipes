
file(
	GLOB_RECURSE
	source_files
	
	*.cpp
)

add_library(
	ocra_control${suffix_type}
	${LIB_TYPE}
	${source_files}
)

install(
	TARGETS ocra_control${suffix_type}
	${install_type} DESTINATION lib
)
