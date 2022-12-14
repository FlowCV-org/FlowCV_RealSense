# RealSense external Library Dependency
IF( DEFINED ENV{REALSENSE_DIR} )
    SET( REALSENSE_DIR "$ENV{REALSENSE_DIR}" )
ELSE()
    find_path(REALSENSE_DIR "REALSENSE_DIR")
ENDIF()

if ("${REALSENSE_DIR}" STREQUAL "REALSENSE_DIR-NOTFOUND")
    if(WIN32)
        message(FATAL_ERROR "No REALSENSE_DIR SDK Variable Found")
    elseif(APPLE)
        message(FATAL_ERROR "No REALSENSE_DIR SDK Variable Found")
    else()
        if(EXISTS /usr/include/librealsense2)
            set(REALSENSE_DIR /usr/include/librealsense2)
            set(REALSENSE_LIB_DIR /usr/lib/x86_64-linux-gnu)
        elseif(EXISTS /usr/local/include/librealsense2)
            set(REALSENSE_DIR /usr/local/include/librealsense2)
            set(REALSENSE_LIB_DIR /usr/local/lib)
        else()
            message(FATAL_ERROR "No RealSense Library Found")
        endif()
    endif()
else()
    set(REALSENSE_LIB_DIR ${REALSENSE_DIR})
endif()

message("REALSENSE SDK Dir: ${REALSENSE_DIR}")
include_directories("${REALSENSE_DIR}/include")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    if(WIN32)
        file(GLOB_RECURSE RS_LIB_DEB ${REALSENSE_LIB_DIR}/realsense2d.lib)
        foreach(lib IN ITEMS ${RS_LIB_REL})
            if(${lib} MATCHES "/x64/")
                set(RS_LIB_REL ${lib})
            endif()
        endforeach()
    elseif(APPLE)
        file(GLOB_RECURSE RS_LIB_DEB ${REALSENSE_LIB_DIR}/librealsense2d.dylib)
    else()
        file(GLOB_RECURSE RS_LIB_DEB ${REALSENSE_LIB_DIR}/librealsense2d.so)
    endif()
    if("${RS_LIB_DEB}" STREQUAL "")
        message(FATAL_ERROR "Could Not Find realsense2d Library")
    else()
        message("RealSense Lib: ${RS_LIB_DEB}")
    endif()
    LIST(APPEND REALSENSE_LIBS ${RS_LIB_DEB})
else()
    if(WIN32)
        file(GLOB_RECURSE RS_LIB_REL ${REALSENSE_LIB_DIR}/realsense2.lib)
        foreach(lib IN ITEMS ${RS_LIB_REL})
            if(${lib} MATCHES "/x64/")
                set(RS_LIB_REL ${lib})
            endif()
        endforeach()
    elseif(APPLE)
        file(GLOB_RECURSE RS_LIB_REL ${REALSENSE_LIB_DIR}/librealsense2.dylib)
    else()
        file(GLOB_RECURSE RS_LIB_REL ${REALSENSE_LIB_DIR}/librealsense2.so)
    endif()
    if("${RS_LIB_REL}" STREQUAL "")
        message(FATAL_ERROR "Could Not Find realsense2 Library")
    else()
        message("RealSense Lib: ${RS_LIB_REL}")
    endif()
    LIST(APPEND REALSENSE_LIBS ${RS_LIB_REL})
endif()
