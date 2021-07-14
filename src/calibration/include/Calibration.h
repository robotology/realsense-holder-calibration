/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <chrono>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <thrift/CalibrationIDL.h>

#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpCameraParameters.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Image.h>

#include <ForwardKinematics.h>


class Calibration : public yarp::os::RFModule,
                    public CalibrationIDL
{
public:

    /*
     * RFModule interface.
     */
    bool configure(yarp::os::ResourceFinder& rf) override;

    bool close() override;

    double getPeriod() override;

    bool updateModule() override;

    /*
     * IDL interface.
     */

    std::string go_home() override;

    std::string quit() override;

    std::string start() override;

    std::string stop() override;

    /* Module state. */
    enum class State{GoHome, Idle, NextPose, Quit, Store, Stop, Wait};

private:

    /* Open robot devices. */
    bool open_remote_control_board(const std::string&, const std::string&);

    /* Retrieve joints configurations for calibration. */
    bool get_joints_configuration(const yarp::os::ResourceFinder& rf);

    /* Retrieve camera parameters. */
    bool get_camera_parameters(const yarp::os::ResourceFinder& rf);

    /* Retrieve the end effector pose. */
    vpPoseVector ee_pose();

    /* Stop robot motion. */
    void stop_motion();

    /* Set module state. */
    void set_state(const Calibration::State& state);

    /* Get module state. */
    Calibration::State get_state();

    /* Devices required to control the robot position. */
    std::unordered_map<std::string, yarp::dev::PolyDriver> drivers_;
    std::unordered_map<std::string, yarp::dev::IControlMode*> control_mode_;
    std::unordered_map<std::string, yarp::dev::IPositionControl*> position_control_;

    /* Joints speeds at startup. */
    std::vector<double> torso_speeds_;
    std::vector<double> head_speeds_;

    /* Timer related. */
    std::chrono::steady_clock::time_point start_time_;

    /* Waiting time set in the configuration file. */
    double wait_time_;

    /* Number of configuration poses. */
    int number_of_poses_;

    /* Counter for number of poses. */
    int counter_poses_ = 0;

    /* Storage for the joints configurations. */
    std::vector<std::vector<double>> joints_;

    /* Storage for camera parameters. */
    vpCameraParameters cam_parameters_;

    /* Forward kinematics. */
    std::unique_ptr<ForwardKinematics> fk_;

    /* Module state. */
    State state_;

    /* RPC port and related. */
    yarp::os::Port port_rpc_;
    std::mutex mutex_;

    /* Buffered Port receiving images from the realsense. */
    yarp::os::BufferedPort <yarp::sig::ImageOf<yarp::sig::PixelRgb>>  port_image_in_;

    /* Name to be used in messages. */
    const std::string log_name_ = "Calibration";
};

#endif /* CALIBRATION_H */
