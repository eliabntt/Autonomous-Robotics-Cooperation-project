//
// Created by rig8f on 09/12/18.
//

#ifndef PROJECT_OBJECTBOX_H
#define PROJECT_OBJECTBOX_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class ObjectBox {
private:
    const double W = 0.297, L = 0.210; // A4 paper size = shape of the box
    std::vector<bool> free = {true, true, true,
                              true, true, true};
    std::vector<geometry_msgs::Pose> poses;
    std::vector<std::string> names;// todo maybe unneeded, to save objects name

public:
    ObjectBox(geometry_msgs::Pose robotPose);
    std::string posOccupiedBy(int pos);
    bool isEmpty();
    bool isFull();

    // bool returns if place is possible, pose contains the actual position
    bool placeCylinder(std::string name, geometry_msgs::Pose &valid);
    bool placeCube(std::string name, geometry_msgs::Pose &valid);
    bool placeTriangle(std::string name, geometry_msgs::Pose &valid);
    // todo maybe implement clearing of last if place is not successful
};

#endif //PROJECT_OBJECTBOX_H
