
SET (LibUVC_FOUND FALSE)
find_path(LibUVC_INCLUDE_DIR libuvc/libuvc.h
    PATHS
        /usr
        /usr/local
    PATH_SUFFIXES
        include
    )

find_library(LibUVC_LIBRARY
    NAMES 
        uvc
    PATHS
        /usr
        /usr/local
    PATH_SUFFIXES
        lib
    )


IF (LibUVC_INCLUDE_DIR AND LibUVC_LIBRARY)
	SET (LibUVC_FOUND TRUE)
ENDIF (LibUVC_INCLUDE_DIR AND LibUVC_LIBRARY)

IF (LibUVC_FOUND)
   MESSAGE(STATUS "Found LibUVC: ${LibUVC_LIBRARY}")
ENDIF (LibUVC_FOUND)


mark_as_advanced(LibUVC_INCLUDE_DIR LibUVC_LIBRARY)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(LibUVC
    REQUIRED_VARS
        LibUVC_LIBRARY
        LibUVC_INCLUDE_DIR
)