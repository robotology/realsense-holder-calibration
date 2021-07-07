#include <chrono>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include "Calibration.h"
#include <thread>
#include <unordered_map>

#include <iCub/iKin/iKinFwd.h>

#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

int main (int argc, char** argv)
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        std::cout<<"YARP doesn't seem to be available"<<std::endl;
        return EXIT_FAILURE;
    }

    Calibration mod;
    yarp::os::ResourceFinder rf;

    //rf.setDefaultContext("test");
    //rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);
    return mod.runModule(rf);

}
