/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Publisher.h>

#include <fstream>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpException.h>

#include <yarp/math/Math.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;


bool Publisher::configure(ResourceFinder &rf)
{
    /* Get the robot name. */
    std::string robot_name = rf.check("robot_name", Value("icub")).asString();

    /* Get the eye version. */
    std::string eye_version = rf.check("eye_version", Value("v2")).asString();

    /* Get the period in seconds. */
    period_ = rf.check("period", Value(0.01)).asDouble();

    /* Get the path to the file containing the calibration matrix. */
    std::string path = rf.check("calibration_file_path", Value("")).asString();
    if (path.empty())
    {
        path = rf.getHomeContextPath() + "/eMc.txt";

        yInfo() << log_name_ + "::configure(). Info: the user did not provide a path for the calibration matrix file (using --calibration_file_path).";
        yInfo() << log_name_ + "::configure(). Info: the calibration file will be searched in " + path  + ".";
    }

    /* Try to open the file containing the calibration matrix. */
    std::ifstream in(path);
    if (!in.is_open())
    {
        yError() << log_name_ + "::configure(). Error: cannot open the calibration matrix file " + path + ".";
        return false;
    }

    /* Try to load the calibration matrix. */
    try
    {
        vpHomogeneousMatrix visp_H;
        visp_H.load(in);

        if (!visp_H.isAnHomogeneousMatrix())
        {
            yError() << log_name_ + "::configure(). The calibration matrix is not a valid homogeneous matrix";

            return false;
        }

        /* Store in a yarp::sig::Matrix for later use. */
        calibration_matrix_.resize(4, 4);
        for (std::size_t i = 0; i < 4; i++)
            for (std::size_t j = 0; j < 4; j++)
                calibration_matrix_(i, j) = visp_H[i][j];

        yInfo() << log_name_ + "::configure(). Info: the loaded calibration matrix is :";
        yInfo() << log_name_ + "::configure(). Info: " + calibration_matrix_.toString();
    }
    catch (const vpException& e)
    {
        yError() << log_name_ + "::configure(). Error while reading the calibration matrix file " + path + ".";
        yError() << log_name_ + "::configure(). Error is: " + e.getStringMessage() + ".";

        return false;
    }

    /* Open output port. */
    if(!port_output_.open("/realsense-holder-publisher/pose:o"))
    {
        yError() << log_name_ + "::ctor(). Error: cannot open the output port.";

        return false;
    }

    /* Configure the forward kinematics. */
    fk_ = std::make_unique<ForwardKinematics>(robot_name, eye_version);

    return true;
}


bool Publisher::close()
{
    /* Close output port. */
    port_output_.close();

    return true;
}


double Publisher::getPeriod()
{
    return period_;
}


bool Publisher::updateModule()
{
    /* Get the forward kinematics up to iCubHeadCenter. */
    Matrix root_to_center = fk_->ee_pose();

    Matrix root_to_camera = root_to_center * calibration_matrix_;

    /* Send to port. */
    Vector axis_angle = dcm2axis(root_to_camera.submatrix(0, 2, 0, 2));

    Vector& pose = port_output_.prepare();
    pose.resize(7);
    pose(0) = root_to_camera(0, 3);
    pose(1) = root_to_camera(1, 3);
    pose(2) = root_to_camera(2, 3);
    pose(3) = axis_angle(0);
    pose(4) = axis_angle(1);
    pose(5) = axis_angle(2);
    pose(6) = axis_angle(3);

    port_output_.write();

    return true;
}
