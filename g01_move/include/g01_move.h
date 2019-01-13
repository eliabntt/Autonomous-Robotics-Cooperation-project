//
// Created by eliabntt on 28/11/18.
//
#include <utility>
#include <ros/ros.h>
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

    // interesting poses
    move_base_msgs::MoveBaseGoal nearCorridor, corridorEntrance, corridorInside, unloadPoint;

    // pose subscription stuff
    geometry_msgs::Pose marrPose, marrPoseOdom;
    ros::Subscriber marrPoseSub, marrPoseOdomSub;

    void subPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL);
    void subPoseOdomCallback(const nav_msgs::Odometry::ConstPtr& msgOdom);
    void poseToYPR(geometry_msgs::Pose pose, double *yaw, double *pitch, double *roll);

    // plan-based moving methods
    bool moveToGoal(move_base_msgs::MoveBaseGoal goal);
    bool inPlaceCW90(move_base_msgs::MoveBaseGoal &pos);
    bool inPlaceCCW90(move_base_msgs::MoveBaseGoal &pos);
    void recoverManual(bool rot = false);

    // corridor part
    ros::Publisher velPub;
    ros::Subscriber scannerSub;

    // distance from front wall, default speeds
    double frontWallDist = 1.05, lateralMinDist = 0.28, linVel = 0.3, twistVel = 0.25;

    // laser callback internal values
    double val, minDx, maxDx, avgDx, minSx, maxSx, avgSx, forwardDist;
    int size,  howMuchDataToUse = 25; // angle span to consider
    geometry_msgs::Twist moveCommand;
    bool isManualModeDone = false;

    // laser-based moving methods
    void wallFollower(bool forward);
    void rotateDX();
    void forwardCallback();
    void backwardCallback();
    void readLaser(const sensor_msgs::LaserScan::ConstPtr &msg);
};

#endif