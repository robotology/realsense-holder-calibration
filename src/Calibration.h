/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */
 #ifndef CALIBRATION_H
 #define CALIBRATION_H

#include <bits/stdc++.h>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <thread>
#include <unordered_map>

#include <iCub/iKin/iKinFwd.h>
#include <stdexcept>
#include <vector>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>


class Calibration : public yarp::os::RFModule
{
private:

    /* Store the name of the robot. */
    std::string robot_name_;

    /* Varibale which describe the kinematic of the straight line coming out from the pooint located between the eyes. */
    iCub::iKin::iCubHeadCenter icub_head_center_;

    /* Initialize devices to control the robot position. */
    std::unordered_map<std::string, yarp::dev::PolyDriver> drivers_;
    std::unordered_map<std::string, yarp::dev::IEncoders*> encoders_;
    std::unordered_map<std::string, yarp::dev::IControlMode*> control_mode_;
    std::unordered_map<std::string, yarp::dev::IPositionControl*> position_control_;

    /* Initialize the time variable. */
    std::chrono::system_clock::time_point start1_;

    /* Storage for the number of configuration poses. */
    int number_of_poses_;

    /* Counter of the poses. */
    int counter_poses_ = 0;

    /* Storage for the poses. */
    std::vector<std::vector<double>> vect_;

    /* Enumeration class and related variable for the state machine states. */
    enum class StateRobot{idle, next_pose, wait_pose};
    StateRobot status_;

    /* Open the Devices. */
    bool OpenRemoteControlboard(const std::string &, const std::string&);

    /* Retrieve the data from the config.ini file. */
    bool RetrieveTheData(yarp::os::Bottle  rf_poses_group);

public:
    
    bool configure(yarp::os::ResourceFinder &rf);

    bool updateModule();

    double getPeriod();

    bool close();



};
#endif /* CALIBRATION_H. */
