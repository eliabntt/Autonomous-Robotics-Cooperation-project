//
// Created by rig8f on 09/12/18.
//

#include <ObjectBox.h>
#include "../../../../../../opt/ros/kinetic/include/tf/LinearMath/Quaternion.h"

/* Scheme of the box
 * --------------> X
 * | 0 | 1 | 2 |
 * | 3 | 4 | 5 |
 * |------------
 * V
 * Y
 */

ObjectBox::ObjectBox(geometry_msgs::Pose robotPose) {
    geometry_msgs::Pose center, temp;
    center.position.z = robotPose.position.z;
    //todo check if necessary
    temp.orientation.x = 0;
    temp.orientation.y = 0;
    temp.orientation.z = 0;
    temp.orientation.w = 1;
    tf::Quaternion rotation(robotPose.orientation.x, robotPose.orientation.y,
                            robotPose.orientation.z, robotPose.orientation.w);
    tf::Vector3 widthVector(W / 3, 0, 0);
    tf::Vector3 lengthVector(L / 4, 0, 0);
    //rotate the shift
    tf::Vector3 centralOffset = tf::quatRotate(rotation, widthVector);
    rotation = rotation * tf::createQuaternionFromYaw(3.14 / 2);
    tf::Vector3 trasversalOffset = tf::quatRotate(rotation, lengthVector);
    for (int i = 0; i < 3; i++) {
        center.position.x = robotPose.position.x - (i - 1) * centralOffset.x();
        center.position.y = robotPose.position.y - (i - 1) * centralOffset.y();
        for (int j = 0; j < 2; j++) {
            temp.position.x = center.position.x - std::pow(-1,j) * trasversalOffset.getX();
            temp.position.y = center.position.y - std::pow(-1,j) * trasversalOffset.getY();
            poses.emplace_back(temp);

            ROS_INFO_STREAM(temp.position);
        }
    }
}

bool ObjectBox::placeCylinder(std::string name, geometry_msgs::Pose &pose) {
    // return first available positions
    if (isFull()) return false;

    // cylinders can go in 0+3, 1+4, 2+5
    int first = 0, second = 3;
    while (first < 3 && second < 6)
        if (free[first] && free[second]) {
            pose = poses[first]; // fixme ERROR
            free[first] = false;
            free[second] = false;
            names.at(first) = name;
            names.at(second) = name;
            //ROS_INFO_STREAM("Pose x: " << pose.position.x << " y: " << pose.position.y << " z: " << pose.position.z);
            return true;
        } else {
            first += 2;
            second += 2;
        }
    return false;
}

bool ObjectBox::placeCube(std::string name, geometry_msgs::Pose &pose) {
    if (isFull()) return false;

    // cubes can go in first available (single) place
    for (int i = 0; i < 6; i++)
        if (free[i]) {
            pose = poses[i];
            free[i] = false;
            names.at(i) = name;
            return true;
        }
    return false;
}

bool ObjectBox::placeTriangle(std::string name, geometry_msgs::Pose &pose) {
    return placeCube(name, pose); // todo maybe do it better
}

std::string ObjectBox::posOccupiedBy(int pos) {
    if (pos > 0 && pos < 6) return names.at(pos);
    else return std::string("");
}

bool ObjectBox::isEmpty() {
    // all true in free vector
    return (std::find(free.begin(), free.end(), false) == free.end());
}

bool ObjectBox::isFull() {
    // all false in free vector
    return (std::find(free.begin(), free.end(), true) == free.end());
}
