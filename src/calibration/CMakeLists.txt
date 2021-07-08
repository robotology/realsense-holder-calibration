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

set(${TARGET_NAME}_THRIFT_HDR thrift/idl.thrift)

yarp_add_idl(${TARGET_NAME}_THRIFT_SRC ${${TARGET_NAME}_THRIFT_HDR})

add_executable(${TARGET_NAME}
               include/Calibration.h
               src/main.cpp
               src/Calibration.cpp
               ${${TARGET_NAME}_THRIFT_SRC}
)

target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${TARGET_NAME}
                      YARP::YARP_dev
                      YARP::YARP_init
                      YARP::YARP_os
                      YARP::YARP_sig
                      ICUB::iKin
)

install(TARGETS ${TARGET_NAME} DESTINATION bin)

yarp_install(FILES app/conf/config.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${TARGET_NAME})
