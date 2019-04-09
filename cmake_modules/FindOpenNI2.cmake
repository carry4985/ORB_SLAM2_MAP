FIND_PATH( OpenNI2_INCLUDE_PATH OpenNI.h
    /usr/include
    /usr/local/include
    /sw/include
    /opt/local/include
    DOC "The directory where OpenNI.h resides")
FIND_LIBRARY( OpenNI2_LIBRARY
    NAMES OpenNI2 openni2
    PATHS
    /usr/lib64
    /usr/lib
    /usr/local/lib64
    /usr/local/lib
    /sw/lib
    /opt/local/lib
    DOC "The OpenNI2 library")

IF (OpenNI2_INCLUDE_PATH)
    SET( OpenNI2_FOUND 1 CACHE STRING "Set to 1 if OpenNI2 is found, 0 otherwise")
ELSE (OpenNI2_INCLUDE_PATH)
    SET( OpenNI2_FOUND 0 CACHE STRING "Set to 1 if OpenNI2 is found, 0 otherwise")
ENDIF (OpenNI2_INCLUDE_PATH)

MARK_AS_ADVANCED( OpenNI2_FOUND )
