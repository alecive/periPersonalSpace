# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(PPS)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 11)

find_package(YARP)
find_package(ICUB)
find_package(ICUBcontrib)


list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
set(CMAKE_INSTALL_PREFIX ${ICUBCONTRIB_INSTALL_PREFIX})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

## add libraries
add_subdirectory(lib)

## then modules
add_subdirectory(modules)

## then apps
add_subdirectory(app)

#icubcontrib_finalize_export(periPersonalSpace)
icubcontrib_add_uninstall_target()

