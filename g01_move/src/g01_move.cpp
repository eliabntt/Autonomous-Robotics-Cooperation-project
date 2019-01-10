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
    ROS_INFO_STREAM("Working in " << ((sim) ? "SIMULATION" : "REAL"));
    marrPoseSub = n.subscribe("/marrtino/amcl_pose", 100, &G01Move::subPoseCallback, this);
    marrPoseOdomSub = n.subscribe("/marrtino/marrtino_base_controller/odom", 100, &G01Move::subPoseOdomCallback, this);
    velPub = n.advertise<geometry_msgs::Twist>("marrtino/move_base/cmd_vel", 1000);
    spinner.start();

    nearCorridor.target_pose.pose.position.x = 0.0;
    nearCorridor.target_pose.pose.position.y = -1.8;
    nearCorridor.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 0), nearCorridor.target_pose.pose.orientation);
    //bool success = moveToGoal(nearCorridor); // fixme corridor debug

    corridorEntrance.target_pose.pose.position.x = 0.6;
    corridorEntrance.target_pose.pose.position.y = -1.1;
    corridorEntrance.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14 / 2), corridorEntrance.target_pose.pose.orientation);
    //success = moveToGoal(corridorEntrance);  // fixme corridor debug

    // wall follower to reach load point
    wallFollower(true);

    // loading
    ros::Duration(2).sleep();

    // rotation and return
    rotateDX();

    // wall follower to exit corridor
    wallFollower(false);

    // move to unload position
    unloadPoint.target_pose.pose.position.x = -1.65;
    unloadPoint.target_pose.pose.position.y = -0.5;
    unloadPoint.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14), unloadPoint.target_pose.pose.orientation);
    //success = moveToGoal(client, unloadPoint); // fixme corridor debug
    ros::shutdown();
}

void G01Move::changePadding(bool increase) {
    ROS_INFO_STREAM((increase ? "RECOVERY: increasing padding" : "RESTORING padding"));
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;
    double_param.name = "footprint_padding";
    //fixme
    double_param.value = (increase ? -0.005 : 0.02);
    conf.doubles.push_back(double_param);
    srv_req.config = conf;
    ros::service::call("/marrtino/move_base/local_costmap/set_parameters", srv_req, srv_resp);
    ros::Duration(0.1).sleep();
}

bool G01Move::moveToGoal(move_base_msgs::MoveBaseGoal goal) {
    spinner.start();
    // wait for the action server to come up
    // MUST leave this to false otherwise spinner will (probably) create conflicts
    MoveBaseClient client("marrtino/move_base", false);

    while (!client.waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the move_base action server to come up");

    bool backed = false;
    bool rotated = false;
    bool padded = true;
    geometry_msgs::Pose currPose = marrPose;
    while (true) {
        goal.target_pose.header.frame_id = "marrtino_map";
        goal.target_pose.header.stamp = ros::Time::now();

        ROS_INFO("Sending goal");
        client.sendGoal(goal);
        /*if (padded) {
            ros::Duration(1).sleep();
            changePadding(false);
        }*/
        client.waitForResult();
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO_STREAM("Move successful");
            spinner.stop();
            return true;
        } else {
            if (currPose.position.x != marrPose.position.x) {
                ROS_INFO_STREAM("Robot has moved somewhere, resetting recovery flags");
                backed = false;
                rotated = false;
                //padded = false;
            }
            if (!padded) {
                //changePadding(true);
                padded = true;
            } else if (!rotated) {
                recoverManual(true);
                rotated = true;
                currPose = marrPose;
            } else if (!backed) {
                recoverManual();
                backed = true;
                currPose = marrPose;
            } else {
                ROS_ERROR_STREAM("Error, robot failed moving");
                spinner.stop();
                return false;
            }
        }
    }
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
        if (avgSx > avgDx)
            moveCommand.angular.z = defZ;
        else
            moveCommand.angular.z = -defZ;
        moveCommand.linear.x = -defX;
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

    if (forward)
        scannerSub = n.subscribe<sensor_msgs::LaserScan>("/marrtino/scan", 2, &G01Move::forwardCallback, this);
    else
        scannerSub = n.subscribe<sensor_msgs::LaserScan>("/marrtino/scan", 2, &G01Move::backwardCallback, this);

    // wait for movement to finish
    while (!isManualModeDone) {} // todo this is crap
    ROS_INFO_STREAM("WALL END");
}

void G01Move::rotateDX() {
    ROS_INFO_STREAM("ROTATION START");

    double r, p, y, tr, tp, ty;
    poseToYPR(marrPoseOdom, &y, &p, &r);
    ROS_INFO_STREAM("R " << r << " P " << p << " Y " << y);
    ty = y;     // current yaw
    ty -= 3.14; // target yaw
    ROS_INFO_STREAM("TY " << ty);

    // rotate until desired yaw is reached
    moveCommand.linear.x = 0.05;
    moveCommand.angular.z = -3 * twistVel;
    while (fabs(y - ty) > 0.1) {
        poseToYPR(marrPoseOdom, &y, &p, &r);
        velPub.publish(moveCommand);
    }
    ROS_INFO_STREAM("ROTATION END");

    // stop
    moveCommand.linear.x = 0.0;
    moveCommand.angular.z = 0.0;
    velPub.publish(moveCommand);
}

void G01Move::forwardCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // do not consider movements when not in the right zone
    if (isManualModeDone) return;

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

    // distance from front wall
    forwardDist = msg->ranges[size / 2];

    if (forwardDist > loadWallDistance) {
        // we need to move forward

        minDx = avgDx;
        minSx = avgSx;
        val = fabs(minSx - minDx);
        //val = fabs(avgSx - avgDx);
        ROS_INFO_STREAM("FW " << forwardDist << " DX " << minDx << " SX " << minSx << " DIFF " << val);
        /*if ((val > threshMin) && (val < threshMax)) {
            // significant difference but not too high
            // into the corridor, stay centered
            if (minSx < minDx) {
                // turn right, there is space
                ROS_INFO_STREAM("CORR: GO DX");
                moveCommand.angular.z = -twistVel;
            } else if (minSx > minDx) {
                // turn left, there is space
                ROS_INFO_STREAM("CORR: GO SX");
                moveCommand.angular.z = +twistVel;
            } else {
                ROS_INFO_STREAM("CORR: AVANTI SAVOIA");
                moveCommand.angular.z = 0.0;
            }
        } else if (val >= threshMax) {*/
        // corridor, follow left wall
        moveCommand.linear.x = defaultVel;
        if (minSx < lateralMinDist) {
            // too near, turn right
            ROS_INFO_STREAM("GO DX");
            moveCommand.angular.z = -twistVel;
        } else if (minSx > 1.1 * lateralMinDist) {
            // too far, turn left
            ROS_INFO_STREAM("GO SX");
            moveCommand.angular.z = +twistVel;
        } else {
            ROS_INFO_STREAM("AVANTI SAVOIA");
            moveCommand.angular.z = 0.0;
        }
        /*} else {
            // negligible difference, go forward
            ROS_INFO_STREAM("AVANTI SAVOIA");
            moveCommand.angular.z = 0.0;
        }*/
        velPub.publish(moveCommand);
    } else {
        // stop marrtino
        moveCommand.linear.x = 0.0;
        moveCommand.angular.z = 0.0;
        velPub.publish(moveCommand);
        isManualModeDone = true;
    }
}

void G01Move::backwardCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // do not consider movements when not in the right zone
    if (isManualModeDone) return;

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
    val = fabs(avgSx-avgDx);

    // distance from front wall
    forwardDist = msg->ranges[size / 2];

    double yPos, r, p, y;
    yPos = marrPoseOdom.position.y;
    poseToYPR(marrPoseOdom, &y, &p, &r);
    ROS_INFO_STREAM("YPOS " << yPos << " Y " << y << " FW " << forwardDist << " DX " << avgDx << " SX " << avgSx);
    if (yPos > 0.22) {
        // large space
        if (val < 0.1) {
            ROS_INFO_STREAM("AVANTI SAVOIA"); // fixme TUNE HERE!!!
            moveCommand.linear.x = defaultVel / 2;
            moveCommand.angular.z = 0;
        } else if(avgSx < avgDx) {
            ROS_INFO_STREAM("GO DX");
            moveCommand.linear.x = 0.1;
            moveCommand.angular.z = -twistVel;
        } else {
            ROS_INFO_STREAM("GO SX");
            moveCommand.linear.x = 0.1;
            moveCommand.angular.z = +twistVel;
        }
    } else if (yPos < 0.22) {
        // corridor
        if (fabs(y + 3.14 / 2) < 0.1) {
            // near aligned, follow left wall
            if (forwardDist > loadWallDistance) {
                // we need to move forward
                moveCommand.linear.x = defaultVel;
                if (minSx < lateralMinDist) {
                    // too near, turn right
                    ROS_INFO_STREAM("GO DX");
                    moveCommand.angular.z = +twistVel;
                } else if (minSx > 1.1 * lateralMinDist) {
                    // too far, turn left
                    ROS_INFO_STREAM("GO SX");
                    moveCommand.angular.z = -twistVel;
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
        } else {
            // misaligned, fast correction
            moveCommand.linear.x = 0.1;
            if (y + 3.14 / 2 < 0) {
                ROS_INFO_STREAM("FAST DX");
                moveCommand.angular.z = +twistVel;
            }else {
                ROS_INFO_STREAM("FAST SX");
                moveCommand.angular.z = -twistVel;
            }
        }
    }
    velPub.publish(moveCommand);
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