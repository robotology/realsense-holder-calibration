
/*
 * Copyright (C) 2021-2023 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef PUBLISHER_YARP_H
#define PUBLISHER_YARP_H

#include <yarp/os/BufferedPort.h>

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>


class PublisherYarp
{
public:

    PublisherYarp(const std::string& port_name);

    ~PublisherYarp();

    void set_transform(const yarp::sig::Matrix& transform);

private:
    /* Output port. */
    yarp::os::BufferedPort<yarp::sig::Vector> port_output_;

    /* Name to be used in messages. */
    const std::string log_name_ = "PublisherYarp";
};

#endif /* PUBLISHER_YARP_H */
