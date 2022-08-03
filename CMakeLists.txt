cmake_minimum_required(VERSION 3.21)
project(FlowCV_Realsense)

set(CMAKE_CXX_STANDARD 17)

include(CMake/RS_Config.cmake)

find_package( FlowCV REQUIRED )

add_subdirectory(RS_Camera)

