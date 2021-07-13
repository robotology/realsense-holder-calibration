/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Calibration.h>

#include <fstream>

#include <opencv2/opencv.hpp>

#include <visp3/core/vpXmlParserCamera.h>

#include <yarp/cv/Cv.h>
#include <yarp/math/Math.h>
#include <yarp/os/Bottle.h>



bool Calibration::configure(yarp::os::ResourceFinder &rf)
{
    /* Get the robot name. */
    std::string robot_name = rf.check("robot_name", yarp::os::Value("icub")).asString();

    /* Get the wait time. */
    wait_time_ = rf.check("wait_time", yarp::os::Value(5.0)).asDouble();

    /* Get the eye version. */
    std::string eye_version = rf.check("eye_version", yarp::os::Value("v2")).asString();

    /* Get the number of poses. */
    yarp::os::Value number_of_poses_value = rf.find("number_of_poses");

    if (number_of_poses_value.isNull())
    {
        std::cerr << log_name_ + "::ctor(). Error: the provided configuration file does not specify the expected number of poses" << std::endl;
        return false;
    }
    else
        number_of_poses_ = number_of_poses_value.asInt();

    /* Retrieve joints configurations for calibration. */
    if (!get_joints_configuration(rf))
        return false;

    /* Retrieve camera parameters. */
    if (!get_camera_parameters(rf))
        return false;

    /* Open RGB input port. */
    port_image_in_.open("/realsense-holder-calibration/rgb:i");

    /* Open torso control board. */
    if (!open_remote_control_board(robot_name, "torso"))
        return false;

    /* Open head control board. */
    if (!open_remote_control_board(robot_name, "head"))
        return false;

    /* Store original speeds. */
    torso_speeds_.resize(3);
    head_speeds_.resize(6);
    position_control_["torso"]->getRefSpeeds(torso_speeds_.data());
    position_control_["head"]->getRefSpeeds(head_speeds_.data());
    std::vector<double> tmp {head_speeds_[0], head_speeds_[1], head_speeds_[2]};
    head_speeds_ = tmp;

    /* Set safe speeds. */
    std::vector<int> joints {0, 1, 2};
    std::vector<double> speeds {5.0, 5.0, 5.0};
    position_control_["torso"]->setRefSpeeds(joints.size(), joints.data(), speeds.data());
    position_control_["head"]->setRefSpeeds(joints.size(), joints.data(), speeds.data());

    /* Open RPC port and attach to respond handler. */
    if (!port_rpc_.open("/realsense-holder-calibration/rpc:i"))
    {
        std::cerr << log_name_ + "::ctor(). Error cannot open input RPC port." << std::endl;
        return false;
    }
    if (!(this->yarp().attachAsServer(port_rpc_)))
    {
        std::cerr << log_name_ << "::configure. Error: cannot attach RPC port to the respond handler." << std::endl;
        return false;
    }

    if (!(this->yarp().attachAsServer(port_rpc_)))
    {
        std::cerr << log_name_ << "::configure. Error: cannot attach RPC port to the respond handler." << std::endl;
        return false;
    }

    /* Configure the forward kinematics. */
    icub_head_center_ = iCub::iKin::iCubHeadCenter("right_" + eye_version);
    icub_head_center_.releaseLink(0);
    icub_head_center_.releaseLink(1);
    icub_head_center_.releaseLink(2);

    /* Set the initial state of the module. */
    state_ = State::Idle;

    return true;
}


bool Calibration::close()
{
    /* Restore original speeds. */
    std::vector<int> joints {0, 1, 2};
    position_control_["torso"]->setRefSpeeds(joints.size(), joints.data(), torso_speeds_.data());
    position_control_["head"]->setRefSpeeds(joints.size(), joints.data(), head_speeds_.data());

    /* Close driver. */
    drivers_.at("torso").close();
    drivers_.at("head").close();
    port_image_in_.close();
    port_rpc_.close();

    return true;
}


double Calibration::getPeriod()
{
    return 0.1;
}


bool Calibration::updateModule()
{
    State state = get_state();

    switch(state)
    {

        case State::GoHome:
        {
            /* Go to the home position. */
            std::vector<int> joints {0, 1, 2};
            std::vector<double> joints_torso {0, 0, 0};
            std::vector<double> joints_head {0, 0, 0};

            position_control_["torso"]->positionMove(joints.size(), joints.data(), joints_torso.data());
            position_control_["head"]->positionMove(joints.size(), joints.data(), joints_head.data());

            set_state(State::Idle);

            /* Reset the counter of the pose. */
            counter_poses_ = 0;

            break;
        }

        case State::Idle:
        {
            /* Do nothing */
            break;
        }

        case State::NextPose:
        {
            /* Store start time, required for the timer. */
            start_time_ = std::chrono::steady_clock::now();

            /* Set the new position of the joints. */
            std::vector<int> joints {0, 1, 2};
            std::vector<double> joints_torso {joints_[counter_poses_][0], joints_[counter_poses_][1], joints_[counter_poses_][2]};
            std::vector<double> joints_head {joints_[counter_poses_][3], joints_[counter_poses_][4], joints_[counter_poses_][5]};

            position_control_["torso"]->positionMove(joints.size(), joints.data(), joints_torso.data());
            position_control_["head"]->positionMove(joints.size(), joints.data(), joints_head.data());

            set_state(State::Wait);

            /* Increment the counter of the pose. */
            counter_poses_++;

            break;
        }

        case State::Quit:
        {
            stop_motion();

            /* This will cause the RFModule to shutdown. */
            return false;
        }

        case State::Store:
        {
            /* Read the image from the port. */
            yarp::sig::ImageOf<yarp::sig::PixelRgb> * image_from_port = port_image_in_.read(false);

            /* Check if the pointer is not null. */
            if (image_from_port != nullptr)
            {
                /* Save the image. */
                cv::Mat cv_image = yarp::cv::toCvMat(*image_from_port);
                cv::imwrite("image-" + std::to_string(counter_poses_) + ".png", cv_image);

                /* Save camera configuration file once. */
                if (counter_poses_ == 1)
                {
                    vpXmlParserCamera xml_camera;
                    xml_camera.save(cam_parameters_, "./camera.xml", "Camera", cv_image.cols, cv_image.rows);
                }
            }
            else
            {
                set_state(State::Store);
                break;
            }

            /* Get the end-effector pose. */
            vpPoseVector H = ee_pose();

            /* Save the pose. */
            H.saveYAML("./pose_fPe_"+ std::to_string(counter_poses_) + ".yaml", H);

            /* Check if the are still configuration to be executed. */
            if (counter_poses_ == number_of_poses_)
                set_state(State::GoHome);
            else
                set_state(State::NextPose);

            break;
        }

        case State::Stop:
        {
            stop_motion();

            set_state(State::Idle);

            break;
        }

        case State::Wait:
        {
            /* Check if wait time elapsed. */
            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time_).count() > wait_time_)
                set_state(State::Store);

            break;
        }
    }

    return true;
}


bool Calibration::open_remote_control_board(const std::string& robot_name, const std::string& part_name)
{
    yarp::os::Property prop;

    /* Set the property of the device. */
    prop.put("device", "remote_controlboard");
    prop.put("local", "/realsense-holder-calibration/" + part_name);
    prop.put("remote", "/" + robot_name + "/" + part_name);

    /* Try to open the driver. */
    if (!drivers_[part_name].open(prop))
    {
        std::cerr << log_name_ + "::open_remote_control_board(). Error: cannot open " + part_name + " driver." << std::endl;
        return false;
    }

    /* Open the views. */
    bool ok = true;
    ok &= drivers_[part_name].view(encoders_[part_name]);
    ok &= drivers_[part_name].view(control_mode_[part_name]);
    ok &= drivers_[part_name].view(position_control_[part_name]);
    if (!ok)
    {
        std::cerr << log_name_ + "::open_remote_control_board(). Error: cannot open " + part_name + " interfaces." << std::endl;
        return false;
    }

    /* Set the control mode for the joints. */
    control_mode_[part_name]->setControlMode(0, VOCAB_CM_POSITION);
    control_mode_[part_name]->setControlMode(1, VOCAB_CM_POSITION);
    control_mode_[part_name]->setControlMode(2, VOCAB_CM_POSITION);

    return true;
}


bool Calibration::get_joints_configuration(const yarp::os::ResourceFinder& rf)
{
    const yarp::os::Bottle& poses_group = rf.findGroup("CALIBRATION_POSES");

    for(std::size_t i = 0; i < number_of_poses_; i++)
    {
        yarp::os::Value configuration = poses_group.find("config_" + std::to_string(i));

        if (configuration.isNull())
        {
            std::cerr << log_name_ + "::get_joints_configuration(). Error: joint configuration " + std::to_string(i) + " is missing." << std::endl;
            return false;
        }

        /* Get the vector related to the i-th configuration. */
        yarp::os::Bottle* b = configuration.asList();
        if (b == nullptr)
        {
            std::cerr << log_name_ + "::get_joints_configuration(). Error: while reading joint configuration " + std::to_string(i) + "." << std::endl;
            return false;
        }

        std::vector<double> entry;
        for (std::size_t j = 0; j < b->size(); j++)
        {
            /* Get the jth value of the single pose. */
            yarp::os::Value item = b->get(j);

            /*Check if the value is null. */
            if (item.isNull())
            {
                std::cerr << log_name_ + "::get_joints_configuration(). Error: cannot read "
                          << std::to_string(j) + "-th element of the "
                          << std::to_string(i) + "-th configuration." << std::endl;
                return false;
            }

            entry.push_back(item.asDouble());
        }

        joints_.push_back(entry);
    }

    return true;
}


bool Calibration::get_camera_parameters(const yarp::os::ResourceFinder& rf)
{
    const yarp::os::Bottle& camera_group = rf.findGroup("CAMERA_INTRINSICS");

    auto get_parameter = [this, camera_group](const std::string& name)
    {
        bool valid = true;
        double parameter;

        yarp::os::Value value = camera_group.find(name);
        if (value.isNull() || (!value.isDouble()))
        {
            valid = false;
            std::cerr << log_name_ + "::get_camera_parameters(). Error: missing or invalid CAMERA_INTRINSICS::" + name + "." << std::endl;
        }
        else
            parameter = value.asDouble();

        return std::make_pair(valid, parameter);
    };

    std::unordered_map<std::string, double> parameters;

    bool valid = true;
    for (auto param_name : {"fx", "fy", "cx", "cy"})
    {
        bool valid_item;
        std::tie(valid_item, parameters[param_name]) = get_parameter(param_name);
        valid &= valid_item;
    }

    if (valid)
    {
        cam_parameters_.initPersProjWithDistortion(parameters["fx"], parameters["fy"], parameters["cx"], parameters["cy"], 0.0, 0.0);

        return true;
    }

    return false;
}


vpPoseVector Calibration::ee_pose()
{
    /* Initialize the vector to store the values of the joints. */
    yarp::sig::Vector chain;

    /* Max number of joints to set. */
    int max_size = 3;

    /* Get the position of the joints. */
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

    yarp::sig::Vector axis_angle = yarp::math::dcm2axis(H.submatrix(0, 2, 0, 2));
    vpPoseVector pose;
    pose.buildFrom
    (
        H(0, 3), H(1, 3), H(2, 3),
        axis_angle[0] * axis_angle[3],
        axis_angle[1] * axis_angle[3],
        axis_angle[2] * axis_angle[3]
    );

    return pose;
}


void Calibration::stop_motion()
{
    std::vector<int> joints {0, 1, 2};

    position_control_["torso"]->stop(joints.size(), joints.data());
    position_control_["head"]->stop(joints.size(), joints.data());
}



void Calibration::set_state(const State& state)
{
    mutex_.lock();
    state_ = state;
    mutex_.unlock();
}


Calibration::State Calibration::get_state()
{
    State state;

    mutex_.lock();
    state = state_;
    mutex_.unlock();

    return state;
}

std::string Calibration::go_home()
{
    /* Return to the home position. */

    set_state(State::GoHome);

    return "Command accepted.";
}

std::string Calibration::quit()
{
    /* Stop robot motion and quit the module. */

    set_state(State::Quit);

    return "Command accepted.";
}


std::string Calibration::start()
{
    /* Start data collection. */

    set_state(State::NextPose);

    return "Command accepted.";
}


std::string Calibration::stop()
{
    /* Immediately stop robot motion. */

    set_state(State::Stop);

    return "Command accepted.";
}
