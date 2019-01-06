//
// Created by eliabntt on 28/11/18.
//
#include <utility>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>

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
    geometry_msgs::Pose marrPose;
    ros::Subscriber marrPoseSub;
    void subPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL);

    // plan-based moving methods
    bool moveToGoal(move_base_msgs::MoveBaseGoal goal);
    bool inPlaceCW90(move_base_msgs::MoveBaseGoal &pos);
    bool inPlaceCCW90(move_base_msgs::MoveBaseGoal &pos);

    // corridor part
    ros::Publisher velPub;
    ros::Subscriber scannerSub;

    ros::Subscriber scannerSubBis;

    double loadWallDistance = 1.05, corrWallDistance = 1.05;
    double defaultVel = 0.15, maxVel = 0.18, lateralMinDist = 0.219,
        threshMin = 0.018, threshMax = 0.08;

    double val, minDx, maxDx, minSx, maxSx, forwardDist, avgL, avgR;
    int size,  howMuchDataToUse = 25; // angle span to consider
    geometry_msgs::Twist moveCommand;
    bool isManualModeDone = false;

    void recoverManual(bool rot = false);
    void wallFollower(bool forward);
    void forwardCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void backwardCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void averageLR(const sensor_msgs::LaserScan::ConstPtr &msg);

    /* fixme useful resources
    http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/
    http://www.theconstructsim.com/ros-qa-126-how-to-configure-the-differential-drive-ros-controller/
    https://answers.ros.org/question/289561/help-to-run-diff_drive_controller/
    https://books.google.it/books?id=skFPDwAAQBAJ&pg=PA182&lpg=PA182&dq=two+wheeled+robot+ros+yaml+config&source=bl&ots=fgpCIFfQSa&sig=90Tbzlv0YMDtm_aLxqukdQby_Q4&hl=en&sa=X&ved=2ahUKEwiYh4CXocrfAhXF-aQKHTjqBvIQ6AEwBXoECAUQAQ#v=onepage&q=two%20wheeled%20robot%20ros%20yaml%20config&f=false

    https://www.youtube.com/watch?v=q3Dn5U3cSWk

    http://wiki.ros.org/navigation/Tutorials/RobotSetup#Costmap_Configuration_.28local_costmap.29_.26_.28global_costmap.29
    https://answers.ros.org/question/279148/costmap-parameters-navigation-stack/
    https://answers.ros.org/question/219108/ghost-obstacles-on-local-costmap/
    https://answers.ros.org/question/277769/how-to-decrease-obstacle-area-in-costmaps/

    https://answers.ros.org/question/268485/navigation-planning-too-near-to-obstacles/
    https://answers.ros.org/question/12874/costmap2d-inflation-radius/

    REPEAT
    1 -> start corridor
     a. -> start wall following https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_WALLFOLLOWING
                                https://github.com/ssscassio/ros-wall-follower-2-wheeled-robot
            since I have only to go straight and to check the distance from the front wall
    2 -> loading
     a. -> get distance from end wall and side wall
     b. -> publish for LZ branch
    3. -> end corridor(rotate and position the robot)
    4. -> start corrdior
    5. -> unloading pose
    */
};

#endif