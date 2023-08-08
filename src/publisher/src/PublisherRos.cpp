/*
 * Copyright (C) 2021-2023 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <PublisherRos.h>

#include <yarp/os/Property.h>

using namespace yarp::os;


PublisherRos::PublisherRos(const std::string& node_name, const std::string& source_frame_id, const std::string& destination_frame_id)
{
    /* Configure the transform setter device. */
    Property device_properties;
    device_properties.put("device", "frameTransformSet_nwc_ros2");

    Property general_properties;
    general_properties.put("period", 1.0 / 100.0);
    general_properties.put("asynch_pub", 0);

    Property ros_properties;
    ros_properties.put("ft_node", node_name);

    device_properties.addGroup("GENERAL") = general_properties;
    device_properties.addGroup("ROS2") = ros_properties;

    bool outcome = true;
    outcome &= driver_.open(device_properties);
    if (!outcome)
        throw(std::runtime_error(log_name_ + "::ctor(). Error: cannot open the polydriver."));

    outcome &= driver_.view(transform_setter_);
    if (!outcome)
        throw(std::runtime_error(log_name_ + "::ctor(). Error: cannot open the IFrameTransformStorageSet view."));

    /* Configure the source and destination frames for the transform. */
    transform_.src_frame_id = source_frame_id;
    transform_.dst_frame_id = destination_frame_id;
}


PublisherRos::~PublisherRos()
{
    if (driver_.isValid())
        driver_.close();
}


void PublisherRos::set_transform(const yarp::sig::Matrix& transform)
{
    /* Update the transform. */
    transform_.fromMatrix(transform);

    /* Update time stamp. */
    stamp_.update();
    transform_.timestamp = stamp_.getTime();

    /* Set the transform. */
    transform_setter_->setTransform(transform_);
}
