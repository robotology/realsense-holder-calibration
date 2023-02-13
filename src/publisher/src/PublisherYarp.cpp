/*
 * Copyright (C) 2021-2023 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <PublisherYarp.h>

#include <yarp/math/Math.h>

using namespace yarp::math;
using namespace yarp::sig;


PublisherYarp::PublisherYarp(const std::string& port_name)
{
    /* Open output port. */
    if(!port_output_.open(port_name))
        throw(std::runtime_error(log_name_ + "::ctor(). Error: cannot open the output port."));
}

PublisherYarp::~PublisherYarp()
{
    /* Close output port if open. */
    if (!port_output_.isClosed())
        port_output_.close();
}


void PublisherYarp::set_transform(const yarp::sig::Matrix& transform)
{
    /* Send to port. */
    Vector axis_angle = dcm2axis(transform.submatrix(0, 2, 0, 2));

    Vector& pose = port_output_.prepare();
    pose.resize(7);
    pose(0) = transform(0, 3);
    pose(1) = transform(1, 3);
    pose(2) = transform(2, 3);
    pose(3) = axis_angle(0);
    pose(4) = axis_angle(1);
    pose(5) = axis_angle(2);
    pose(6) = axis_angle(3);

    port_output_.write();
}