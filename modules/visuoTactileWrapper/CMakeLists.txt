# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(visuoTactileWrapper)

file(GLOB source *.cpp)
file(GLOB header *.h)

include_directories(${periPersonalSpace_INCLUDE_DIRS})

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(visuoTactileWrapper ${source} ${header})

target_link_libraries(visuoTactileWrapper periPersonalSpace ctrlLib ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS visuoTactileWrapper DESTINATION bin)

