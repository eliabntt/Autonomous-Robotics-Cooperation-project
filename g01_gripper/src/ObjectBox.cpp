//
// Created by rig8f on 09/12/18.
//

#include <ObjectBox.h>
#include <tf2_ros/transform_listener.h>

/* Scheme of the box
 * --------------> X
 * | 0 | 1 | 2 |
 * | 3 | 4 | 5 |
 * |------------
 * V
 * Y
 */

ObjectBox::ObjectBox() {
    names.resize(6);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf2_listener(tfBuffer);
    geometry_msgs::TransformStamped odom_to_world;
    odom_to_world = tfBuffer.lookupTransform("world", "marrtino_odom", ros::Time(0), ros::Duration(10.0) );
    std::vector<geometry_msgs::Pose> newPoses;
    //ROS_INFO_STREAM(robotPose.position.x);
    double OFFSETX = - H/2; // todo tune signs
    double OFFSETY = + W/2;
    double OFFSETZ = 1;
    for (int r = 1; r < 4; r += 2) {
        for (int c = 1; c < 6; c += 2) {
            geometry_msgs::Pose pose;
            pose.position.x = OFFSETX + (double) c / 6 * W;
            pose.position.y = OFFSETY + (double) r / 4 * H;
            pose.position.z = OFFSETZ;
            pose.orientation.w = 1;
            tf2::doTransform(pose, pose, odom_to_world);
            newPoses.emplace_back(pose);
            /*ROS_INFO_STREAM(
                    "Creation: r: " << r << " c: " << c << " pose: x: " << pose.position.x << " y: " << pose.position.y
                                    << " z: " << pose.position.z);*/
        }
    }
    poses = newPoses;
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
