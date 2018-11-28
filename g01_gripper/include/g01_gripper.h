//
// Created by eliabntt on 28/11/18.
//
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

//ROS messages
#include <robotiq_s_model_control/SModel_robot_output.h>
#include <robotiq_s_model_control/SModel_robot_input.h>

#ifndef G01_GRIPPER_G01_GRIPPER_H
#define G01_GRIPPER_G01_GRIPPER_H

class G01Gripper{
public:
    //Default constructor
    G01Gripper();


private:
    void close(int howMuch);
    void open();

    moveit_msgs::CollisionObject collision(geometry_msgs::Pose pose, double length, double width, double height, std::string obj_id);


    ros::NodeHandle n;

    //gripper
    robotiq_s_model_control::SModel_robot_output command;
    robotiq_s_model_control::SModel_robot_input status;

    //collisions
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher gripperCommandPub;
    ros::ServiceClient client;
    bool sim;

};

#endif