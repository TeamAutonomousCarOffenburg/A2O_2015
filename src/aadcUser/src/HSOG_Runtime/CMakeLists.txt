cmake_minimum_required(VERSION 2.8)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O0 -Wall -fPIC" )

set(A2O_LIB "a2ol")
set(A2O_TOOLS_PROJECT "a2o_toolsp")

add_subdirectory(tools)

add_subdirectory(a2o)
#add_subdirectory(tests)
add_subdirectory(adtf)

