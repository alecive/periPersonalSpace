# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(ultimateTracker)

file(GLOB source *.cpp)
file(GLOB header *.h)

include_directories(${periPersonalSpace_INCLUDE_DIRS})

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(ultimateTracker ${source} ${header})

target_link_libraries(ultimateTracker periPersonalSpace ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS ultimateTracker DESTINATION bin)

