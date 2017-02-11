cmake_minimum_required (VERSION 3.1)
project (nbody)

set (NBODY_SRCS
	simulation.cpp
	orbit_integration.cpp
	orbit_integration.h
	datastructures.h)

# g++ requires selection of C++ standard
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()
	
add_executable(nbody ${NBODY_SRCS})