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
    move_base_msgs::MoveBaseGoal nearCorridor, corridorEntrance, loadPoint, unloadPoint;

    //Pose subscription stuff
    geometry_msgs::Pose marrPose, marrPoseOdom;
    ros::Subscriber marrPoseSub;
    ros::Subscriber marrPoseOdomSub;

    void subPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL);
    void subPoseOdomCallback(const nav_msgs::Odometry::ConstPtr& msgOdom);
    void poseToYPR(geometry_msgs::Pose pose, double *yaw, double *pitch, double *roll);

    // plan-based moving methods
    bool moveToGoal(move_base_msgs::MoveBaseGoal goal);
    bool inPlaceCW90(move_base_msgs::MoveBaseGoal &pos);
    bool inPlaceCCW90(move_base_msgs::MoveBaseGoal &pos);
    void changePadding(bool increase);

    // corridor part
    ros::Publisher velPub;
    ros::Subscriber scannerSub;

    double loadWallDistance = 1.05, corrWallDistance = 1.05;
    double defaultVel = 0.3, twistVel = 0.22, lateralMinDist = 0.28,
        threshMin = 0.01, threshMax = 0.08;

    double val, minDx, maxDx, minSx, maxSx, forwardDist, avgSx, avgDx;
    int size,  howMuchDataToUse = 25; // angle span to consider
    geometry_msgs::Twist moveCommand;
    bool isManualModeDone = false;

    void recoverManual(bool rot = false);
    void wallFollower(bool forward);
    void rotateDX();
    void forwardCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void backwardCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
};

#endif