cmake_minimum_required (VERSION 3.1)
project (nbody)

set (NBODY_SRCS
	simulation.cpp
	orbit_integration.cpp
	orbit_integration.h
	datastructures.h)
	
add_executable(nbody ${NBODY_SRCS})