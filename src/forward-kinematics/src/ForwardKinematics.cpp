/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ForwardKinematics.h>

#include <iostream>

#include <yarp/os/Network.h>
#include <yarp/sig/Matrix.h>


ForwardKinematics::ForwardKinematics(const std::string& robot_name, const std::string& head_version)
{
    /* Check network. */
    if (!yarp_.checkNetwork())
        throw(std::runtime_error(log_name_ + "::ctor(). YARP network is not available."));

    /* Configure head and torso encoders. */
    for (const auto& part_name : {"torso", "head"})
        if (!configure_encoders(robot_name, part_name))
            throw(std::runtime_error(log_name_ + "::ctor(). Cannot configure encoders for " + robot_name + "/" + part_name));

    /* Configure the forward kinematics. */
    icub_head_center_ = iCub::iKin::iCubHeadCenter("right_" + head_version);
    icub_head_center_.releaseLink(0);
    icub_head_center_.releaseLink(1);
    icub_head_center_.releaseLink(2);
}


ForwardKinematics::~ForwardKinematics()
{
    /* Close drivers. */
    drivers_.at("torso").close();
    drivers_.at("head").close();
}


yarp::sig::Matrix ForwardKinematics::ee_pose()
{
    /* Initialize the vector to store the values of the joints. */
    yarp::sig::Vector chain;

    /* Max number of joints to set. */
    int max_size = 3;

    /* Get the configuration of the joints. */
    for (const std::string part_name : {"torso", "head"})
    {
        int size;
        encoders_[part_name]->getAxes(&size);

        yarp::sig::Vector part_chain(size);
        encoders_[part_name]->getEncoders(part_chain.data());
        int offset = 0;
        if (part_name == "torso")
            offset = -2;

        for (int i = 0; i < max_size; i++)
            chain.push_back(part_chain(abs(offset + i)));
    }

    /* Evaluate the forward kinematics. */
    yarp::sig::Matrix H = icub_head_center_.getH(chain * M_PI / 180.0);

    return H;
}


bool ForwardKinematics::configure_encoders(const std::string& robot_name, const std::string& part_name)
{
    yarp::os::Property prop;

    /* Set the property of the device. */
    prop.put("device", "remote_controlboard");
    prop.put("local", "/realsense-holder-calibration/forward_kinematics/" + part_name);
    prop.put("remote", "/" + robot_name + "/" + part_name);

    /* Try to open the driver. */
    if (!drivers_[part_name].open(prop))
    {
        std::cerr << log_name_ + "::open_remote_control_board(). Error: cannot open " + part_name + " driver." << std::endl;
        return false;
    }

    /* Try to get the IEncoder interface. */
    if (!drivers_[part_name].view(encoders_[part_name]))
    {
        std::cerr << "::open_remote_control_board(). Error: cannot open " + part_name + " IEncoder interface." << std::endl;
        return false;
    }

    return true;
}
