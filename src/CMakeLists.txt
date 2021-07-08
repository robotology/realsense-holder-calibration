#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(TARGET_NAME realsense-holder-calibration)

find_package(YARP REQUIRED COMPONENTS dev os sig)

find_package(ICUB REQUIRED COMPONENTS iKin)

add_executable(${TARGET_NAME} main.cpp Calibration.cpp Calibration.h)

target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)

target_link_libraries(${TARGET_NAME}
                      YARP::YARP_dev
                      YARP::YARP_init
                      YARP::YARP_os
                      YARP::YARP_sig
                      ICUB::iKin
)

install(TARGETS ${TARGET_NAME} DESTINATION bin)

yarp_install(FILES ${CMAKE_SOURCE_DIR}/app/conf/config.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${TARGET_NAME})
