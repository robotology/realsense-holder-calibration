/*
 * Copyright (C) 2021-2023 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef PUBLISHER_ROS_H
#define PUBLISHER_ROS_H

#include <yarp/dev/IFrameTransformStorage.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/math/FrameTransform.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>


class PublisherRos
{
public:

    PublisherRos(const std::string& node_name, const std::string& source_frame_id, const std::string& destination_frame_id);

    ~PublisherRos();

    void set_transform(const yarp::sig::Matrix& transform);

private:
    /* PolyDriver . */
    yarp::dev::PolyDriver driver_;

    /* IFrameTransformStorageSet interface. */
    yarp::dev::IFrameTransformStorageSet* transform_setter_;

    /* Storage for the transform. */
    yarp::math::FrameTransform transform_;

    /* Storage for the time stamp. */
    yarp::os::Stamp stamp_;

    /* Name to be used in messages. */
    const std::string log_name_ = "PublisherRos";
};

#endif /* PUBLISHER_ROS_H */
