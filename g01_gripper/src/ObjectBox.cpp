//
// Created by rig8f on 09/12/18.
//

#include <ObjectBox.h>

/* Scheme of the box
 * --------------> X
 * | 0 | 1 | 2 |
 * | 3 | 4 | 5 |
 * |------------
 * V
 * Y
 */

ObjectBox::ObjectBox(geometry_msgs::Pose robotPose) {
    names.resize(6);
    ROS_INFO_STREAM("OB robotPose: x: " << robotPose.position.x << " y: " << robotPose.position.y << " z: " << robotPose.position.z);

    // set box offset w.r.t. world frame using given robot pose
    // robot position is assumed to be centered
    // offset refers to upper-left corner of box (corner of 0)
    double OFFSETX = robotPose.position.x - W / 2; // todo tune signs
    double OFFSETY = robotPose.position.y - H / 2;
    double OFFSETZ = 0.2;

    // construct poses (positions in box)
    // 1/6w 1/4h  3/6w 1/4h  5/6w 1/4h
    // 1/6w 3/4h  3/6w 3/4h  5/6w 3/4h
    for (int r = 1; r < 4; r += 2)
        for (int c = 1; c < 6; c += 2) {
            geometry_msgs::Pose pose;
            pose.position.x = OFFSETX + (double) c / 6 * W;
            pose.position.y = OFFSETY + (double) r / 4 * H;
            pose.position.z = robotPose.position.z + OFFSETZ;
            pose.orientation = robotPose.orientation;
            poses.emplace_back(pose);
            ROS_INFO_STREAM("Creation: r: " << r << " c: " << c << " pose: x: " << pose.position.x << " y: " << pose.position.y << " z: " << pose.position.z);

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
            ROS_INFO_STREAM("Pose x: " << pose.position.x << " y: " << pose.position.y << " z: " << pose.position.z);
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
