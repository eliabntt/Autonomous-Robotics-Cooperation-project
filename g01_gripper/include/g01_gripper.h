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
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_ros_link_attacher/AttachRequest.h"
#include "gazebo_ros_link_attacher/AttachResponse.h"

#ifndef G01_GRIPPER_G01_GRIPPER_H
#define G01_GRIPPER_G01_GRIPPER_H

class G01Gripper {
public:
    //Default constructor
    G01Gripper();

private:
    bool sim;

    ros::NodeHandle n;
    ros::Subscriber gripperStatusSub;
    ros::Publisher gripperCommandPub;

    //gripper
    robotiq_s_model_control::SModel_robot_output command;
    robotiq_s_model_control::SModel_robot_input status;

    void getGripper(const robotiq_s_model_control::SModel_robot_input &msg);

    ros::ServiceClient attacher, detacher;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::vector<geometry_msgs::PoseStamped> objectsToAvoid, cylToGrab, cubeToGrab, triToGrab; //,objectsToGrab;

    //movement
    std::string PLANNING_GROUP = "manipulator";

    void moveObjects(moveit::planning_interface::MoveGroupInterface &group,
                     std::vector<geometry_msgs::PoseStamped> objectList, bool rotate = false);

    bool move(geometry_msgs::Pose destination, moveit::planning_interface::MoveGroupInterface &group);
    std::vector<geometry_msgs::Pose> makeWaypoints(geometry_msgs::Pose from, geometry_msgs::Pose to,
                                                   unsigned long n_steps = 3);

    std::vector<double> home_joint_positions{-3.1415 / 2, -1.86, 3.1415/2, -3.1415/2, -3.1415 / 2, 0};

    void poseToYPR(geometry_msgs::Pose pose, double *yaw, double *pitch, double *roll);

    // collisions
    moveit_msgs::CollisionObject
    addCollisionBlock(geometry_msgs::Pose pose, float Xlen, float Ylen, float Zlen, std::string obj_id);

    moveit_msgs::CollisionObject removeCollisionBlock(std::string obj_id);
    void addCollisionWalls();

    // apriltags connection
    ros::Subscriber subGrab, subAvoid;

    void grabCB(const g01_perception::PoseStampedArray::ConstPtr &input);

    void avoidCB(const g01_perception::PoseStampedArray::ConstPtr &input);

    bool gazeboAttach(std::string model2, std::string link2);

    bool gazeboDetach(std::string model2, std::string link2);

    // gripper
    void gripperClose(int howMuch);

    void gripperOpen();

    bool isHeld();
};

#endif