//
// Created by rig8f on 09/12/18.
//

#include <ObjectBox.h>

/* Scheme of the box
 * Map coordinate/world
 * --------------> X / Y
 * | 0 | 2 | 4 |
 * | 1 | 3 | 5 |
 * |------------
 * V
 * Y / X
 */
ObjectBox::ObjectBox(geometry_msgs::Pose robotPose) {
    geometry_msgs::Pose center, temp;
    temp.orientation.w = 1;
    tf::Quaternion rotation(robotPose.orientation.x, robotPose.orientation.y,
                            robotPose.orientation.z, robotPose.orientation.w);
    robotPose.position.y -= 0.02;//todo tune
    robotPose.position.x += 0.005;
    tf::Vector3 widthVector(W / 3, 0, 0);
    tf::Vector3 lengthVector(L / 4, 0, 0);
    //rotate the shift
    rotation = rotation * tf::createQuaternionFromYaw(3.14 / 2);
    tf::Vector3 centralOffset = tf::quatRotate(rotation, widthVector);
    rotation = rotation * tf::createQuaternionFromYaw(3.14 / 2);
    tf::Vector3 trasversalOffset = tf::quatRotate(rotation, lengthVector);

    for (int i = 0; i < 3; i++) {
        center.position.x = robotPose.position.x - (i - 1) * centralOffset.x();
        center.position.y = robotPose.position.y - (i - 1) * centralOffset.y();
        center.position.z = 0.7;
        tf::quaternionTFToMsg(rotation, center.orientation);
        possiblePoses.emplace_back(center);
        for (int j = 0; j < 2; j++) {
            temp.position.x = center.position.x - std::pow(-1, j) * trasversalOffset.x();
            temp.position.y = center.position.y - std::pow(-1, j) * trasversalOffset.y();
            temp.position.z = 0.7;
            temp.orientation = center.orientation;
            poses.emplace_back(temp);
        }
    }
}

int ObjectBox::poseToIndex(const geometry_msgs::Pose &pose) {
    int i = 0;
    for (auto p: poses) {
        if (p.position.x == pose.position.x && p.position.y == pose.position.y)
            return i;
        i += 1;
    }
    return -1;
}

bool ObjectBox::getCylinderPose(geometry_msgs::Pose &output) {
    // return first available positions
    if (isFull()) return false;

    // cylinders can go in 0+1, 2+3, 4+5
    int first = 0, second = 1;
    while (first <= 4 && second <= 5)
        if (free[first] && free[second]) {
            output = poses[first];
            return true;
        } else {
            first += 2;
            second += 2;
        }
    return false;
}

bool ObjectBox::markCylinderOcc(geometry_msgs::Pose &pose) {
    // get index of given pose in the array
    int index = poseToIndex(pose);
    if (!free[index] && !free[index + 1])
        return false;

    // mark as occupied
    free[index] = false;
    free[index + 1] = false;
    return true;
}

bool ObjectBox::getCubePose(geometry_msgs::Pose &output, bool * indexEven) {
    if (isFull()) return false;

    // cubes can go in first available (single) place
    for (int i = 0; i < 6; i++)
        if (free[i]) {
            output = poses[i];
            *indexEven = i%2==0;
            return true;
        }
    return false;
}

bool ObjectBox::markCubeOcc(geometry_msgs::Pose &pose) {
    // get index of given pose in the array
    int index = poseToIndex(pose);
    if (!free[index])
        return false;

    // mark as occupied
    free[index] = false;
    return true;
}

bool ObjectBox::isEmpty() {
    // all true in free vector
    return (std::find(free.begin(), free.end(), false) == free.end());
}

bool ObjectBox::isFull() {
    // all false in free vector
    return (std::find(free.begin(), free.end(), true) == free.end());
}
