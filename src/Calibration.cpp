/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include "Calibration.h"


bool Calibration::configure(yarp::os::ResourceFinder &rf)
{
    /* Get the robot name. */
    robot_name_ = rf.check("icubSim", yarp::os::Value("icubSim")).asString();

    /* Check if the robot is the driver are open for the torso. */
    if(!Calibration::OpenRemoteControlboard(robot_name_, "torso"))
    {
        return EXIT_FAILURE;
    }

    /* Check if the robot is the driver are open for the head. */
    if(!Calibration::OpenRemoteControlboard(robot_name_, "head"))
    {
        return EXIT_FAILURE;
    }

    /* Release the links. */
    icub_head_center_.releaseLink(0);
    icub_head_center_.releaseLink(1);
    icub_head_center_.releaseLink(2);

    /* Store the number of different poses necessary for the calibration. */
    number_of_poses_ = rf.find("number_of_poses").asInt();

    /* Point the group that contains the absolute angles of the joints. */
    yarp::os::Bottle rf_poses_group = rf.findGroup("CALIBRATION_POSES");

    /* Store the data from the config.ini file*/
    Calibration::RetrieveTheData(rf_poses_group);

    /* Set the initial state of the state machine. */
    this->status_ = StateRobot::next_pose;

    return true;
}

bool Calibration::RetrieveTheData(yarp::os::Bottle rf_poses_group)
{
    for(int i = 0; i<number_of_poses_; i++)
    {

        if (rf_poses_group.find("Config"+std::to_string(i)).isNull())
        {
            throw(std::runtime_error("The key does not exist"));
            return false;
        }

        /* Get the vector related to the ith pose. */
        yarp::os::Bottle * b = rf_poses_group.find("Config"+std::to_string(i)).asList();

        /* Temporary vector to store the data. */
        std::vector<double> temp;

        /* Check if the vector is nulla. */
        if(b == nullptr)
        {

            throw(std::runtime_error("The vector is empty"));
            return false

        }
        for (std::size_t j=0; j<b->size(); j++)
        {
            /* Get the jth value of the single pose. */
            yarp::os::Value item = b->get(j);

            /*Check if the value is null. */
            if (item.isNull())
            {
                throw(std::runtime_error("The value of the " +  std::to_string(j) + " element of the " +  std::to_string(i) + "configure is null"));
                return false;
            }

            /* Push the value into the temporary vector. */
            temp.push_back( item.asDouble());
        }

        /*Push the temporary vector into the class variable vect_. */
        vect_.push_back(temp);

    }

    return true;

}

bool Calibration::OpenRemoteControlboard(const std::string & robot, const std::string & part_name)
{
    yarp::os::Property prop;

    /* Set the property of the device. */
    prop.put("device", "remote_controlboard");
    prop.put("local", "/realsense-holder-calibration/" + part_name);
    prop.put("remote", "/" + robot_name_ + "/" + part_name);

    /* Check if the driver is open. */
    if (!drivers_[part_name].open(prop))
    {
        std::cout << "Cannot open " + part_name + " driver." << std::endl;

        return false;
    }

    /* Open the views. */
    bool ok= true;
    ok = ok && this->drivers_[part_name].view(this->encoders_[part_name]);
    ok = ok && this->drivers_[part_name].view(this->control_mode_[part_name]);
    ok = ok && this->drivers_[part_name].view(this->position_control_[part_name]);

    /* Check if the views are open. */
    if (!ok)
    {
            throw(std::runtime_error("Unable to open views"));
    }

    /* Set the control mode for the joints. */
    this->control_mode_[part_name]->setControlMode(0, VOCAB_CM_POSITION);
    this->control_mode_[part_name]->setControlMode(1, VOCAB_CM_POSITION);
    this->control_mode_[part_name]->setControlMode(2, VOCAB_CM_POSITION);

    return true;


}


double Calibration::getPeriod()
{
    return 1.0;
}

bool Calibration::updateModule()
{


    switch(this->status_)
    {
        case StateRobot::idle:
        {

            std::cout <<" Calibration done! " << std::endl;
            return true;

        }
        case StateRobot::next_pose:
        {
            /* Store the time value from when we wait for 5 seconds. */
            start1_ = std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now());

            /*Set the new position of the joints. */
            this->position_control_["torso"]->positionMove(0, vect_[counter_poses_][0]);
            this->position_control_["torso"]->positionMove(1, vect_[counter_poses_][1]);
            this->position_control_["torso"]->positionMove(2, vect_[counter_poses_][2]);
            this->position_control_["head"]->positionMove(0, vect_[counter_poses_][3]);
            this->position_control_["head"]->positionMove(1, vect_[counter_poses_][4]);
            this->position_control_["head"]->positionMove(2, vect_[counter_poses_][5]);

            /* Set the next state of the state machine. */
            this->status_ = StateRobot::wait_pose;

        }
        case StateRobot::wait_pose:
        {
            /* Check if 5 seconds have passed. */
            if ( std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()  - start1_).count() >= 5 )
            {
                /* Initialize the vector to store the values of the joints. */
                yarp::sig::Vector chain;

                /* Max number of joints to set. */
                int max_size = 3;

                /*Get the position of the joints. */
                for (const std::string part_name : {"torso", "head"})
                {
                    int size;
                    encoders_[part_name]->getAxes(&size);

                    yarp::sig::Vector part_chain(size);
                    encoders_[part_name]->getEncoders(part_chain.data());

                    int offset = 0;
                    if (part_name == "torso")
                        offset = -2;

                    for (std::size_t i = 0; i < max_size; i++)
                        chain.push_back(part_chain(abs(offset + i)));
                }


                std::cout<<"Loading done."<<std::endl;

                /* Convert the values into a matrix wrt the reference system. */
                yarp::sig::Matrix H = this->icub_head_center_.getH(chain * M_PI / 180.0);

                /* Increment the counter of the pose. */
                counter_poses_ ++;

                /* Open the file where to store the pose. */
                std::ofstream pose_file;
                pose_file.open("pose_fPe_" + std::to_string(counter_poses_) + ".yaml");
                std::cout <<" Creating pose file number " + std::to_string(counter_poses_) +"..."<< std::endl;
                pose_file << H.toString() << std::endl;

                /* Close the file. */
                pose_file.close();

                /* Check if the are any other configuration poses and set the next state. */
                if (counter_poses_ < number_of_poses_)
                {
                    this->status_ = StateRobot::next_pose;
                }
                else
                {
                    this->status_ = StateRobot::idle;
                }
            }

            /* Wait for the 5 seconds to end. */
            else
            {
                std::cout<<"Loading..."<<std::endl;
            }
        }

    }



    return true;
}
bool Calibration::close()
{
    return true;
}
