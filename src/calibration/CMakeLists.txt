#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(TARGET_NAME realsense-holder-calibration)

find_package(OpenCV REQUIRED)

find_package(VISP REQUIRED)

find_package(YARP REQUIRED COMPONENTS dev os sig cv)

set(${TARGET_NAME}_THRIFT_HDR thrift/idl.thrift)

yarp_add_idl(${TARGET_NAME}_THRIFT_SRC ${${TARGET_NAME}_THRIFT_HDR})
add_executable(${TARGET_NAME}
               include/Calibration.h
               src/main.cpp
               src/Calibration.cpp
               ${${TARGET_NAME}_THRIFT_SRC}
)

target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${VISP_INCLUDE_DIRS})

target_link_libraries(${TARGET_NAME}
                      realsense-holder-forward-kinematics
                      YARP::YARP_cv
                      YARP::YARP_dev
                      YARP::YARP_init
                      YARP::YARP_os
                      YARP::YARP_sig
                      ${VISP_LIBRARIES}
                      ${OpenCV_LIBS}
)

install(TARGETS ${TARGET_NAME} DESTINATION bin)
install(PROGRAMS scripts/calibrate.sh DESTINATION bin RENAME realsense-holder-calibration-process)

yarp_install(FILES app/realsense-holder-calibration.xml DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES
             app/conf/config.ini
             app/conf/config_iCubGenova01_holder_tilt.ini
             app/conf/config_iCubGenova01_holder_no_tilt.ini
             app/conf/config_rs.ini
             DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${TARGET_NAME}
)
