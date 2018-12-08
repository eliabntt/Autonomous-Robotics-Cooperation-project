//
// Created by eliabntt on 28/11/18.
//
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <robotiq_s_model_control/SModel_robot_output.h>
#include <robotiq_s_model_control/SModel_robot_input.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_ros_link_attacher/AttachRequest.h>
#include <gazebo_ros_link_attacher/AttachResponse.h>
#include "g01_perception/PoseStampedArray.h"
#include "../../g01_perception/include/tags.h"

#ifndef G01_GRIPPER_G01_GRIPPER_H
#define G01_GRIPPER_G01_GRIPPER_H

class G01Gripper {
public:
    //Default constructor
    G01Gripper();

private:
    // gripper
    void gripperClose(int howMuch);
    void gripperOpen(); // todo if command is mostly 0s, init here and set only the relevant on the method
    bool gazeboAttach(std::string name, std::string link);
    bool gazeboDetach(std::string name, std::string link);

    // manipulator
    std::vector<double> HOME_JOINT_POS{-3.1415 / 2, -3.1415 / 2, 3.1415 / 2, -3.1415 / 2, -3.1415 / 2, 0};
    void moveObjects(moveit::planning_interface::MoveGroupInterface &group,
                     std::vector<geometry_msgs::PoseStamped> objectList, bool rotate = false);
    bool moveManipulator(geometry_msgs::Pose destination, moveit::planning_interface::MoveGroupInterface &group);
    std::vector<geometry_msgs::Pose> makeWaypoints(geometry_msgs::Pose from, geometry_msgs::Pose to,
                                                   unsigned long nSteps = 3);

    // collisions
    moveit_msgs::CollisionObject addCollisionBlock(geometry_msgs::Pose pose,
                                                   float Xlen, float Ylen, float Zlen, std::string objectId,
                                                   bool isTriangle = false);
    void addCollisionWalls();
    void grabCB(const g01_perception::PoseStampedArray::ConstPtr &input);
    void avoidCB(const g01_perception::PoseStampedArray::ConstPtr &input);

    // utilities
    void poseToYPR(geometry_msgs::Pose pose, double *yaw, double *pitch, double *roll);
    void gripperCB(const robotiq_s_model_control::SModel_robot_input &msg);
    bool isHeld();

    // members
    bool sim;
    ros::NodeHandle n;
    std::string PLANNING_GROUP = "manipulator";

    // subscriber
    ros::Subscriber gripperStatusSub;
    ros::Subscriber subGrab, subAvoid;
    robotiq_s_model_control::SModel_robot_input status;

    // publisher
    ros::ServiceClient attacher, detacher;
    ros::Publisher gripperCommandPub;
    robotiq_s_model_control::SModel_robot_output command;

    // objects to track
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::vector<std::string> grabObjNames;
    std::vector<geometry_msgs::PoseStamped> objectsToAvoid, cylToGrab, cubeToGrab, triToGrab;
};

#endif