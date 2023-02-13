
/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include <yarp/sig/Matrix.h>

#include <ForwardKinematics.h>
#include <PublisherYarp.h>


class Publisher : public yarp::os::RFModule
{
public:

    /*
     * RFModule interface.
     */
    bool configure(yarp::os::ResourceFinder& rf) override;

    bool close() override;

    double getPeriod() override;

    bool updateModule() override;

private:

    /* iCub forward kinematics. */
    std::unique_ptr<ForwardKinematics> fk_;

    /* Output via YARP. */
    std::unique_ptr<PublisherYarp> output_yarp_;

    /* Period. */
    double period_;

    /* Calibratrion matrix. */
    yarp::sig::Matrix calibration_matrix_;

    /* Name to be used in messages. */
    const std::string log_name_ = "Publisher";
};

#endif /* PUBLISHER_H */
