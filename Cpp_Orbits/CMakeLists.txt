cmake_minimum_required (VERSION 3.1)
project (nbody)

set (NBODY_SRCS
	simulation.cpp
	orbit_integration.cpp)
	
add_executable(nbody ${NBODY_SRCS})