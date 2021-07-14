#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(TARGET_NAME realsense-holder-forward-kinematics)

find_package(YARP REQUIRED COMPONENTS dev os sig cv)

find_package(ICUB REQUIRED COMPONENTS iKin)

add_library(${TARGET_NAME}
            include/ForwardKinematics.h
            src/ForwardKinematics.cpp
)

target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${TARGET_NAME}
                      PRIVATE
                      YARP::YARP_dev
                      YARP::YARP_init
                      YARP::YARP_os
                      YARP::YARP_sig
                      ICUB::iKin
)
