#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(TARGET_NAME realsense-holder-publisher)

find_package(VISP REQUIRED)

find_package(YARP REQUIRED COMPONENTS os sig)

add_executable(${TARGET_NAME}
               src/main.cpp
               include/Publisher.h
               include/PublisherRos.h
               include/PublisherYarp.h
               src/Publisher.cpp
               src/PublisherRos.cpp
               src/PublisherYarp.cpp
)

target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${VISP_INCLUDE_DIRS})

target_link_libraries(${TARGET_NAME}
                      realsense-holder-forward-kinematics
                      YARP::YARP_init
                      YARP::YARP_os
                      YARP::YARP_sig
                      ${VISP_LIBRARIES}
)

install(TARGETS ${TARGET_NAME} DESTINATION bin)

yarp_install(FILES app/realsense-holder-publisher.xml DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES app/conf/config.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${TARGET_NAME})
