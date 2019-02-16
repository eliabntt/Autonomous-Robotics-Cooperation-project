//
// Created by rig8f on 09/12/18.
//

#ifndef PROJECT_OBJECTBOX_H
#define PROJECT_OBJECTBOX_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class ObjectBox {
public:
    const double W = 0.317, L = 0.210; // A4 paper size = shape of the box
    std::vector<bool> free = {true, true, true,
                              true, true, true};

    std::vector<geometry_msgs::Pose> poses, possiblePoses;
    ObjectBox(geometry_msgs::Pose robotPose);
    bool isEmpty();
    bool isFull();

    int poseToIndex(const geometry_msgs::Pose &pose);

    // bool returns if place is possible, pose contains the actual position
    bool getCylinderPose(geometry_msgs::Pose &output);
    bool markCylinderOcc(geometry_msgs::Pose &pose);
    bool getCubePose(geometry_msgs::Pose &output, bool *indexEven);
    bool markCubeOcc(geometry_msgs::Pose &pose);
};

#endif //PROJECT_OBJECTBOX_H
