//
// Created by rig8f on 26/12/18.
//
#include "g01_move.h"

G01Move::G01Move() : n(), spinner(2) {
    ros::NodeHandle nh("~");
    if (nh.hasParam("sim") && (!nh.getParam("sim", sim))) {
        ROS_WARN_STREAM("Invalid value for 'sim' parameter. Setting 'sim' to 'true'.");
        sim = true;
    }
    spinner.start();

    ROS_INFO_STREAM("Working in " << ((sim) ? "SIMULATION" : "REAL"));
    marrPoseSub = n.subscribe("/marrtino/amcl_pose", 100, &G01Move::subPoseCallback, this);
    marrPoseOdomSub = n.subscribe("/marrtino/marrtino_base_controller/odom", 100, &G01Move::subPoseOdomCallback, this);
    velPub = n.advertise<geometry_msgs::Twist>("marrtino/move_base/cmd_vel", 1000);
    scannerSub = n.subscribe<sensor_msgs::LaserScan>("/marrtino/scan", 2, &G01Move::readLaser, this);


    nearCorridor.target_pose.pose.position.x = 0.0;
    nearCorridor.target_pose.pose.position.y = -1.8;
    nearCorridor.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 0), nearCorridor.target_pose.pose.orientation);
    bool success = moveToGoal(nearCorridor); // fixme corridor debug

    corridorEntrance.target_pose.pose.position.x = 0.4;
    corridorEntrance.target_pose.pose.position.y = -1.6;
    corridorEntrance.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 0.4 * 3.14), corridorEntrance.target_pose.pose.orientation);
    success = moveToGoal(corridorEntrance);  // fixme corridor debug

    corridorInside.target_pose.pose.position.x = 0.6;
    corridorInside.target_pose.pose.position.y = -1.3;
    corridorInside.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14 / 2), corridorInside.target_pose.pose.orientation);
    success = moveToGoal(corridorInside);

    // wall follower to reach load point
    wallFollower(true);

    // loading
    ros::Duration(2).sleep();

    // rotation and return
    rotateDX();

    // wall follower to exit corridor
    wallFollower(false);

    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14 + 0.4 * 3.14),
                          corridorEntrance.target_pose.pose.orientation);
    success = moveToGoal(corridorEntrance);  // fixme corridor debug

    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14), nearCorridor.target_pose.pose.orientation);
    success = moveToGoal(nearCorridor); // fixme corridor debug

    // move to unload position
    unloadPoint.target_pose.pose.position.x = -1.57;
    unloadPoint.target_pose.pose.position.y = -0.37;
    unloadPoint.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14 / 2), unloadPoint.target_pose.pose.orientation);
    success = moveToGoal(unloadPoint); // fixme corridor debug
    spinner.stop();
    ros::shutdown();
}

bool G01Move::moveToGoal(move_base_msgs::MoveBaseGoal goal) {
    // wait for the action server to come up
    // MUST leave this to false otherwise spinner will (probably) create conflicts
    MoveBaseClient client("marrtino/move_base", false);

    while (!client.waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the move_base action server to come up");

    bool backed = false;
    bool rotated = false;
    geometry_msgs::Pose currPose = marrPose;
    while (true) {
        goal.target_pose.header.frame_id = "marrtino_map";
        goal.target_pose.header.stamp = ros::Time::now();

        ROS_INFO("Sending goal");
        client.sendGoal(goal);
        client.waitForResult();
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO_STREAM("Move successful");
            return true;
        } else {
            if (currPose.position.x != marrPose.position.x) {
                ROS_INFO_STREAM("Robot has moved somewhere, resetting recovery flags");
                backed = false;
                rotated = false;
                changeVel(false);
            }

            if (!rotated) {
                recoverManual(true);
                rotated = true;
                currPose = marrPose;
            } else if (!backed) {
                recoverManual();
                backed = true;
                currPose = marrPose;
            } else {
                ROS_ERROR_STREAM("Error, robot failed moving");
                return false;
            }
        }
    }
}

void G01Move::changeVel(bool negative) {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "min_vel_x";
    double_param.value = negative ? -0.15 : 0.1;
    conf.doubles.push_back(double_param);
    srv_req.config = conf;
    ros::service::call("/marrtino/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

void G01Move::recoverManual(bool rot) {
    // initialize topics

    //resetting previous moveCommand settings
    //todo refine
    moveCommand.linear.x = 0;
    moveCommand.angular.z = 0;
    double defZ = 0.4, defX = 0.1;

    if (rot) {
        ROS_INFO_STREAM("Backing up rotating");
        moveCommand.linear.x = defX;
        if (avgSx > avgDx) {
            moveCommand.angular.z = defZ;
            if (avgDx < 0.22) {
                moveCommand.linear.x = -defX;
                moveCommand.angular.z = -defZ;
            }
        } else {
            moveCommand.angular.z = -defZ;
            if (avgSx < 0.22) {
                moveCommand.linear.x = -defX;
                moveCommand.angular.z = defZ;
            }
        }
    } else {
        ROS_INFO_STREAM("Backing up linear");

        tf::Quaternion rotation(marrPoseOdom.orientation.x, marrPoseOdom.orientation.y, marrPoseOdom.orientation.z,
                                marrPoseOdom.orientation.w);
        tf::Vector3 vector(0.2, 0, 0);
        tf::Vector3 rotated_vector = tf::quatRotate(rotation, vector);

        changeVel(true);
        move_base_msgs::MoveBaseGoal goal_temp;
        goal_temp.target_pose.header.frame_id = "marrtino_map";
        goal_temp.target_pose.header.stamp = ros::Time::now();
        goal_temp.target_pose.pose = marrPoseOdom;
        goal_temp.target_pose.pose.position.x -= rotated_vector.getX();
        goal_temp.target_pose.pose.position.y -= rotated_vector.getY();

        ROS_INFO_STREAM("x " << marrPoseOdom.position.x << " y " << marrPoseOdom.position.y << " wanted "
                             << goal_temp.target_pose.pose.position.x << " " << goal_temp.target_pose.pose.position.y);

        MoveBaseClient client_temp("marrtino/move_base", false);
        ROS_INFO("Sending goal");
        client_temp.sendGoal(goal_temp);
        client_temp.waitForResult();

        if (client_temp.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            changeVel(false);
        }
    }
    velPub.publish(moveCommand);
    ros::Duration(2).sleep();
}

bool G01Move::inPlaceCW90(move_base_msgs::MoveBaseGoal &pos) {
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, -3.14 / 2), pos.target_pose.pose.orientation);
}

bool G01Move::inPlaceCCW90(move_base_msgs::MoveBaseGoal &pos) {
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14 / 2), pos.target_pose.pose.orientation);
}

void G01Move::wallFollower(bool forward) { // todo tests needed here
    ROS_INFO_STREAM("WALL START");
    isManualModeDone = false;
    while (!isManualModeDone)
        if (forward)
            forwardCallback();
        else
            backwardCallback();
}

void G01Move::rotateDX() {
    ROS_INFO_STREAM("ROTATION START");

    double r, p, y, ty;
    poseToYPR(marrPoseOdom, &y, &p, &r);
    ty = y;     // current yaw
    ty -= 3.14; // target yaw
    ROS_INFO_STREAM("R " << r << " P " << p << " Y " << y << " TY " << ty);

    // rotate until desired yaw is reached
    moveCommand.linear.x = 0.15;
    moveCommand.angular.z = -4 * twistVel;
    while (fabs(y - ty) > 0.1) {
        velPub.publish(moveCommand);
        ros::Duration(0.1).sleep();
        poseToYPR(marrPoseOdom, &y, &p, &r);
    }

    // stop
    moveCommand.linear.x = 0.0;
    moveCommand.angular.z = 0.0;
    velPub.publish(moveCommand);
    ROS_INFO_STREAM("ROTATION END");
}

void G01Move::readLaser(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // select values from the whole range
    size = (int) msg->intensities.size();
    avgDx = 0;
    avgSx = 0;
    minDx = msg->ranges[0];
    maxDx = msg->ranges[0];
    minSx = msg->ranges[size - 1];
    maxSx = msg->ranges[size - 1];

    // scan and save min avg and max for both sides
    // scan direction is right to left
    for (int i = 0; i < howMuchDataToUse; i++) {
        val = msg->ranges[i];
        if (val < minDx) minDx = val;
        if (val > maxDx) maxDx = val;
        avgDx += val;
    }
    for (int i = size - 1; i > size - 1 - howMuchDataToUse; i--) {
        val = msg->ranges[i];
        if (val < minSx) minSx = val;
        if (val > maxSx) maxSx = val;
        avgSx += val;
    }
    avgDx /= howMuchDataToUse;
    avgSx /= howMuchDataToUse;
    val = fabs(avgSx - avgDx);

    // distance from front wall
    forwardDist = msg->ranges[size / 2];
}

void G01Move::forwardCallback() {
    ROS_INFO_STREAM("FW " << forwardDist << " DX " << avgDx << " SX " << avgSx << " DIFF " << val);

    // fixme do not consider forwardDist if not properly aligned (corr entrance: it stops very early)
    // corridor, follow left wall
    if (forwardDist > frontWallDist) {
        // we need to move forward
        moveCommand.linear.x = linVel;
        if (avgSx < lateralMinDist) {
            // too near, turn right
            ROS_INFO_STREAM("GO DX");
            moveCommand.angular.z = -twistVel;
        } else if (avgSx > 1.1 * lateralMinDist) {
            // too far, turn left
            ROS_INFO_STREAM("GO SX");
            moveCommand.angular.z = +twistVel;
        } else {
            ROS_INFO_STREAM("AVANTI SAVOIA");
            moveCommand.angular.z = 0.0;
        }
    } else if (marrPoseOdom.position.y < 1.5) {
        moveCommand.linear.x = linVel;
        if (avgSx < lateralMinDist) {
            // too near, turn right
            ROS_INFO_STREAM("GO DX");
            moveCommand.angular.z = -2 * twistVel;
        } else if (avgSx > 1.1 * lateralMinDist) {
            // too far, turn left
            ROS_INFO_STREAM("GO SX");
            moveCommand.angular.z = +2 * twistVel;
        } else {
            ROS_INFO_STREAM("AVANTI SAVOIA");
            moveCommand.angular.z = 0.0;
        }
    } else {
        // stop
        moveCommand.linear.x = 0.0;
        moveCommand.angular.z = 0.0;
        isManualModeDone = true;
    }
    velPub.publish(moveCommand);
}

void G01Move::backwardCallback() {
    // position from odom
    double yPos, r, p, y;
    yPos = marrPoseOdom.position.y;
    poseToYPR(marrPoseOdom, &y, &p, &r);
    ROS_INFO_STREAM("YPOS " << yPos << " Y " << y << " FW " << forwardDist << " DX " << avgDx << " SX " << avgSx);

    if (yPos > 0.2) {
        //fixme better!!
        // large space
        moveCommand.linear.x = 0.1;
        if (avgSx < 1.15 * lateralMinDist) {
            ROS_INFO_STREAM("LS GO DX");
            moveCommand.angular.z = -4 * twistVel;
        } else if (avgDx < 1.15 * lateralMinDist) {
            ROS_INFO_STREAM("LS GO SX");
            moveCommand.angular.z = +4 * twistVel;
        } else if (val < 0.4) {
            ROS_INFO_STREAM("LS AVANTI SAVOIA");
            moveCommand.angular.z = 0;
        }
        // fixme quite unstable in the change but then ok
    } else {
        // corridor, follow left wall
        if (forwardDist > frontWallDist) {
            // we need to move forward
            moveCommand.linear.x = linVel;
            if (minSx < lateralMinDist) {
                // too near, turn right
                ROS_INFO_STREAM("GO DX");
                moveCommand.angular.z = -twistVel;
            } else if (minSx > 1.05 * lateralMinDist) {
                // too far, turn left
                ROS_INFO_STREAM("GO SX");
                moveCommand.angular.z = +twistVel;
            } else {
                ROS_INFO_STREAM("AVANTI SAVOIA");
                moveCommand.angular.z = 0.0;
            }
        } else if (yPos > -1.2) {
            moveCommand.linear.x = linVel;
            if (minSx < lateralMinDist) {
                // too near, turn right
                ROS_INFO_STREAM("GO DX");
                moveCommand.angular.z = -2 * twistVel;
            } else if (minSx > 1.05 * lateralMinDist) {
                // too far, turn left
                ROS_INFO_STREAM("GO SX");
                moveCommand.angular.z = +2 * twistVel;
            } else {
                ROS_INFO_STREAM("AVANTI SAVOIA");
                moveCommand.angular.z = 0.0;
            }
        } else {     // stop
            moveCommand.linear.x = 0.0;
            moveCommand.angular.z = 0.0;
            isManualModeDone = true;
        }
    }
    velPub.publish(moveCommand);
    ros::Duration(0.1).sleep();
}

//

void G01Move::subPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL) {
    marrPose = msgAMCL->pose.pose;
}

void G01Move::subPoseOdomCallback(const nav_msgs::Odometry::ConstPtr &msgOdom) {
    marrPoseOdom = msgOdom->pose.pose;
}

void G01Move::poseToYPR(geometry_msgs::Pose pose, double *yaw, double *pitch, double *roll) {
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(pose.orientation, quaternion);
    tf::Matrix3x3 mat(quaternion);
    mat.getEulerYPR(*yaw, *pitch, *roll);
}