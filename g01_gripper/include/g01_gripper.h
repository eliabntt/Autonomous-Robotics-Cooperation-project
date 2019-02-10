//
// Created by eliabntt on 28/11/18.
//
#include <utility>
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <nav_msgs/Odometry.h>
#include <robotiq_s_model_control/SModel_robot_output.h>
#include <robotiq_s_model_control/SModel_robot_input.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_ros_link_attacher/AttachRequest.h>
#include <gazebo_ros_link_attacher/AttachResponse.h>
#include "g01_perception/PoseStampedArray.h"
#include "g01_perception/tags.h"
#include "g01_move/states.h"
#include "ObjectBox.h"

#ifndef G01_GRIPPER_G01_GRIPPER_H
#define G01_GRIPPER_G01_GRIPPER_H

class G01Gripper {
public:
    G01Gripper();

private:
    // state
    unsigned short currState;
    ros::Publisher statePub;
    ros::Subscriber stateSub;
    std_msgs::UInt16 stateCommand;
    void stateCallback(const std_msgs::UInt16::ConstPtr &msg);

    // gripper
    void gripperClose(int howMuch);
    void gripperOpen();
    void fakeGripperClose();
    void fakeGripperOpen();
    bool gazeboAttach(std::string name, std::string link);
    bool gazeboDetach(std::string name, std::string link);

    // manipulator
    std::vector<geometry_msgs::PoseStamped> moveObjects(moveit::planning_interface::MoveGroupInterface &group,
                     std::vector<geometry_msgs::PoseStamped> objectList, ObjectBox & box, bool rotate = false);
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
    void goHome(moveit::planning_interface::MoveGroupInterface &group);
    void goOverLZ(moveit::planning_interface::MoveGroupInterface &group);
    bool isHeld(int howMuch);

    //marrtino pose callback
    void marrOdomCallback(const nav_msgs::Odometry::ConstPtr &OdomPose);

    // MEMBERS
    bool sim, odomReceived = false, full = false, finish = false;
    ros::NodeHandle n;

    // manipulator
    std::string PLANNING_GROUP = "manipulator";
    double PI = 3.1415926535897931;

    // joints' angles for fixed points  base    shoulder   elbow     wr1      wr2      wr3
    std::vector<double> HOME_JOINT_POS {PI / 2, -PI / 2,   -PI / 2,  -PI / 2, PI / 2,  0};
    std::vector<double> LZ_JOINT_POS   {-PI,    -PI / 2.8, PI / 2.8, -PI / 2, -PI / 2, 0};
    geometry_msgs::Pose LZPose;
    geometry_msgs::Pose initialPose; // not in joints here
    std::string planFrameId, endEffId;

    // subscriber
    ros::Subscriber gripperStatusSub;
    ros::Subscriber subGrab, subAvoid;
    ros::Subscriber marrOdomSub;
    robotiq_s_model_control::SModel_robot_input status;

    // publisher
    ros::ServiceClient attacher, detacher;
    ros::Publisher gripperCommandPub, fakeGripperCommandPub;
    robotiq_s_model_control::SModel_robot_output command;

    // objects to track
    moveit::planning_interface::PlanningSceneInterface planningSceneIF;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    std::vector<moveit_msgs::CollisionObject> collObjects;
    std::vector<std::string> grabObjNames;
    std::vector<geometry_msgs::PoseStamped> objectsToAvoid, cylToGrab, cubeToGrab, triToGrab;
};

#endif