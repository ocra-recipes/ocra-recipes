# This module defines
# OCRA_FRAMEWORK_LIBRARY,
# OCRA_FRAMEWORK_FOUND, 
# OCRA_FRAMEWORK_INCLUDE_DIR, where to find the headers
#
# $OCRA_FRAMEWORK_DIR is an environment variable that would
# correspond to the ./configure --prefix=$OCRA_FRAMEWORK_DIR
#
# Copied by Joseph Salini from the FindOSG.cmake, which was
# Created by Robert Osfield. 


if(MSVC)
	set(suffix_type "_static")
else()
	set(suffix_type "")
endif()


FIND_PATH(OCRA_FRAMEWORK_INCLUDE_DIR ocra/optim/ocra_events.h
    ${OCRA_FRAMEWORK_DIR}/include
    C:/usr/local/include
    C:/usr/include
)


FIND_LIBRARY(OCRA_OPTIM_LIBRARY NAMES ocra_optim${suffix_type}
	PATHS
	${OCRA_FRAMEWORK_DIR}/lib
    C:/usr/local/lib
	C:/usr/lib
)

FIND_LIBRARY(OCRA_CONTROL_LIBRARY NAMES ocra_control${suffix_type}
	PATHS
	${OCRA_FRAMEWORK_DIR}/lib
    C:/usr/local/lib
	C:/usr/lib
)


SET(OCRA_FRAMEWORK_FOUND "NO")
IF(OCRA_OPTIM_LIBRARY AND OCRA_CONTROL_LIBRARY AND OCRA_FRAMEWORK_INCLUDE_DIR)
    SET(OCRA_FRAMEWORK_FOUND "YES")
    message(STATUS "found Orc_framework")
ENDIF(OCRA_OPTIM_LIBRARY AND OCRA_CONTROL_LIBRARY AND OCRA_FRAMEWORK_INCLUDE_DIR)

