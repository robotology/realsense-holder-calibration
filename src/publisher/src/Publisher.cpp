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

#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;


bool Publisher::configure(ResourceFinder &rf)
{
    /* Get the robot name. */
    std::string robot_name = rf.check("robot_name", Value("icub")).asString();

    /* Get the eye version. */
    std::string eye_version = rf.check("eye_version", Value("v2")).asString();

    /* Get the period in seconds. */
    period_ = rf.check("period", Value(0.01)).asFloat64();

    /* Get the optional ROS publishing flag. */
    publish_ros_ = rf.check("enable_ros", Value(false)).asBool();
    std::string src_frame_id;
    std::string dst_frame_id;
    if (publish_ros_)
    {
        src_frame_id = rf.check("ros_src_frame_id", Value("icub_root")).asString();
        dst_frame_id = rf.check("ros_dst_frame_id", Value("icub_realsense")).asString();
    }

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

    /* Configure the forward kinematics. */
    fk_ = std::make_unique<ForwardKinematics>(robot_name, eye_version);

    /* Configure output via YARP. */
    output_yarp_ = std::make_unique<PublisherYarp>("/realsense-holder-publisher/pose:o");

    /* Configure output via ROS. */
    if (publish_ros_)
        output_ros_ = std::make_unique<PublisherRos>("realsense_holder_publisher", src_frame_id, dst_frame_id);

    return true;
}


bool Publisher::close()
{
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

    output_yarp_->set_transform(root_to_camera);
    if (publish_ros_)
        output_ros_->set_transform(root_to_camera);

    return true;
}
