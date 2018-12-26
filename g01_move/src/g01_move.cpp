
//
// Created by rig8f on 26/12/18.
//
#include "g01_move.h"

G01Move::G01Move() : n() {
    //fixme maybe 4 better
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle nh("~");
    if (nh.hasParam("sim") && (!nh.getParam("sim", sim))) {
        ROS_WARN_STREAM("Invalid value for 'sim' parameter. Setting 'sim' to 'true'.");
        sim = true;
    }
    ROS_INFO_STREAM("Working in " << ((sim) ? "SIMULATION" : "REAL"));

    // todo content here

    spinner.stop();
    ros::shutdown();
}
