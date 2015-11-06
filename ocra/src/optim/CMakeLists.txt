

file(
	GLOB_RECURSE
	source_files
	
	*.cpp
)

add_library(
	ocra_optim${suffix_type}
	${LIB_TYPE}
	${source_files}
)

install(
	TARGETS ocra_optim${suffix_type}
	${install_type} DESTINATION lib
)