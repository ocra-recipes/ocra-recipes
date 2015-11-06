##### Make library
INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/include
	${PC_EIGEN_INCLUDE_DIRS}
	)

add_library(
    quadprog${suffix_type}
    "QuadProg++.cpp"
)

install(
    TARGETS quadprog${suffix_type}
    ${install_type} DESTINATION lib
)
