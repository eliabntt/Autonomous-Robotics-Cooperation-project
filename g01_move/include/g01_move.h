//
// Created by eliabntt on 28/11/18.
//
#include <utility>
#include <ros/ros.h>

#ifndef G01_MOVE_G01_MOVE_H
#define G01_MOVE_G01_MOVE_H

class G01Move {
public:
    G01Move();

private:
    // MEMBERS
    bool sim;
    ros::NodeHandle n;

};

#endif