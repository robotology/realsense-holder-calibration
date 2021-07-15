/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <cstdlib>
#include <iostream>

#include <Calibration.h>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;


int main (int argc, char** argv)
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << "YARP network not available.";

        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setDefaultContext("realsense-holder-calibration");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);

    Calibration module;
    return module.runModule(rf);
}
