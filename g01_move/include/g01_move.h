//
// Created by eliabntt on 28/11/18.
//
#include <utility>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include "g01_challenge/states.h"

#ifndef G01_MOVE_G01_MOVE_H
#define G01_MOVE_G01_MOVE_H

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class G01Move {
public:
    G01Move();

private:
    // MEMBERS
    bool sim;
    ros::NodeHandle n;
    ros::AsyncSpinner spinner;

    // state
    bool proceed = true, anotherRoundNeeded = true;
    unsigned short currState;
    ros::Publisher statePub;
    ros::Subscriber stateSub;
    ros::Subscriber startSub;
    std_msgs::UInt16 stateCommand;
    void stateCallback(const std_msgs::UInt16::ConstPtr &msg);
    void startCallback(const std_msgs::Bool::ConstPtr &msg);

    // interesting poses
    move_base_msgs::MoveBaseGoal nearCorridor, corridorEntrance, corridorInside, plannerGoal, nearUnloadPoint, unloadPoint;

    // pose subscription stuff
    geometry_msgs::Pose marrPoseOdom, marrPoseAmcl;
    geometry_msgs::PoseWithCovariance marrPose;
    ros::Subscriber marrPoseSub, marrPoseOdomSub;
    ros::ServiceClient clearMapsClient;
    std_srvs::Empty empty;

    void subPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL);
    void subPoseOdomCallback(const nav_msgs::Odometry::ConstPtr &msgOdom);
    void poseToYPR(geometry_msgs::Pose pose, double *yaw, double *pitch, double *roll);

    // plan-based moving methods
    bool moveToGoal(move_base_msgs::MoveBaseGoal goal);
    void recoverManual(bool rot = false);
    void changeVel(bool negative);
    void changeAnglePrec(bool increase);

    // corridor part
    ros::Publisher velPub;
    ros::Subscriber scannerSub;

    // distance from front wall, default speeds
    double frontWallDist, lateralMinDist = 0.3, linVel = 0.4, twistVel = 0.4;
    double PI = 3.1415926535897931;

    // laser callback internal values
    double readThr = 0.6, val, minDx, maxDx, avgDx, minSx, maxSx, avgSx, forwardDist;
    int validCount, size, readIStart, readIEnd, readIFront, howMuchDataToUse = 25; // angle span to consider
    geometry_msgs::Twist moveCommand;
    bool success, isManualModeDone = false, isNearLoadPoint = false, firstRun = true;

    // laser-based moving methods
    void wallFollower(bool forward);
    void docking();
    void rotateRight();
    void deviateRight();
    void followerCallback(bool forward = true);
    void readLaser(const sensor_msgs::LaserScan::ConstPtr &msg);
};

#endif