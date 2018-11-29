//
// Created by eliabntt on 28/11/18.
//
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <robotiq_s_model_control/SModel_robot_output.h>
#include <robotiq_s_model_control/SModel_robot_input.h>
#include "g01_perception/PoseStampedArray.h"
#include "../../g01_perception/include/tags.h"

#ifndef G01_GRIPPER_G01_GRIPPER_H
#define G01_GRIPPER_G01_GRIPPER_H

class G01Gripper {
public:
    //Default constructor
    G01Gripper();

private:
    bool sim;

    ros::NodeHandle n;

    //gripper
    robotiq_s_model_control::SModel_robot_output command;
    robotiq_s_model_control::SModel_robot_input status;
    ros::Publisher gripperCommandPub;
    ros::ServiceClient client;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::vector<geometry_msgs::PoseStamped> objectsToGrab, objectsToAvoid;

    // arm
    bool move(double x, double y);

    // gripper
    void gripperClose(int howMuch);
    void gripperOpen();

    // collisions
    moveit_msgs::CollisionObject addCollisionBlock(geometry_msgs::Pose pose, double Xlen, double Ylen, double Zlen, std::string obj_id);
    moveit_msgs::CollisionObject removeCollisionBlock(std::string obj_id);


    // apriltags connection
    void getApriltagDetections();
    void grabCB(const g01_perception::PoseStampedArray::ConstPtr &input);
    void avoidCB(const g01_perception::PoseStampedArray::ConstPtr &input);
};

#endif