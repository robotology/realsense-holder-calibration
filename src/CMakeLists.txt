#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(TARGET_NAME calibration)

find_package(YARP REQUIRED COMPONENTS dev os sig)

find_package(ICUB REQUIRED COMPONENTS iKin)

add_executable(${TARGET_NAME} main.cpp Calibration.cpp Calibration.h)

target_link_libraries(${TARGET_NAME}
                      YARP::YARP_dev
                      YARP::YARP_init
                      YARP::YARP_os
                      YARP::YARP_sig
                      ICUB::iKin)
