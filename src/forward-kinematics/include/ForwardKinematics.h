/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef FORWARDKINEMATICS_H
#define FORWARDKINEMATICS_H

#include <unordered_map>

#include <iCub/iKin/iKinFwd.h>

#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>


class ForwardKinematics
{
public:

    ForwardKinematics(const std::string& robot_name, const std::string& head_version);

    ~ForwardKinematics();

    /* Retrieve the end effector pose. */
    yarp::sig::Matrix ee_pose();

private:

    /* Configure IEncoders for the given robot part. */
    bool configure_encoders(const std::string& robot_name, const std::string& part_name);

    /* Forward kinematics up to iCubHeadCenter. */
    iCub::iKin::iCubHeadCenter icub_head_center_;

    /* Devices required to read the joints position. */
    std::unordered_map<std::string, yarp::dev::PolyDriver> drivers_;
    std::unordered_map<std::string, yarp::dev::IEncoders*> encoders_;

    /* YARP network. */
    yarp::os::Network yarp_;

    /* Name to be used in log messages. */
    const std::string log_name_ = "ForwardKinematics";
};

#endif /* FORWARDKINEMATICS_H */
