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
    scannerSub = n.subscribe<sensor_msgs::LaserScan>("/marrtino/scan", 2, &G01Move::readLaser, this);
    velPub = n.advertise<geometry_msgs::Twist>("marrtino/move_base/cmd_vel", 10);
    spinner.start();

    nearCorridor.target_pose.pose.position.x = 0.1;
    nearCorridor.target_pose.pose.position.y = -1.8;
    nearCorridor.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14 / 4), nearCorridor.target_pose.pose.orientation);
    success = moveToGoal(nearCorridor);

    corridorEntrance.target_pose.pose.position.x = 0.4;
    corridorEntrance.target_pose.pose.position.y = -1.4;
    corridorEntrance.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 0.4 * 3.14), corridorEntrance.target_pose.pose.orientation);
    success = moveToGoal(corridorEntrance);

    corridorInside.target_pose.pose.position.x = 0.65;
    //todo move near entrance
    corridorInside.target_pose.pose.position.y = -1.3;
    corridorInside.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14 * 0.45), corridorInside.target_pose.pose.orientation);
    success = moveToGoal(corridorInside);

    double r, p, y, ty;
    poseToYPR(marrPoseOdom, &y, &p, &r);
    ROS_INFO_STREAM("y " << marrPoseOdom.position.y);
    ROS_INFO_STREAM("yaw " << y);
    ros::Duration(3).sleep();
    // wall follower to reach load point
    if (marrPoseOdom.position.y < -1.45)
        moveCommand.linear.x = 0.5;
    else
        moveCommand.linear.x = 0;
    if (fabs(1.413 - y) > 0.2) {
        if ((1.413 - y) > 0) {
            ROS_INFO_STREAM("turn left????");//fixme
            moveCommand.angular.z = -0.1;
        } else {
            ROS_INFO_STREAM("turn right????");//fixme
            moveCommand.angular.z = 0.1;
        }
    } else
        moveCommand.angular.z = 0;
    velPub.publish(moveCommand);

    ros::Duration(0.5).sleep(); // to get accurate odom data
    wallFollower(true);

    // loading
    ros::Duration(2).sleep();


    // pose other corr entrance

    //changeInflation(true);
    ROS_INFO_STREAM("ROTATION");

    rotateDX();
    ros::Duration(2).sleep();
    ROS_INFO_STREAM("TO FINAL");
    unloadPoint.target_pose.pose.position.x = 0.92;
    unloadPoint.target_pose.pose.position.y = 0.78;
    unloadPoint.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14 + 3.14 / 3), unloadPoint.target_pose.pose.orientation);
    success = moveToGoal(unloadPoint);

    unloadPoint.target_pose.pose.position.x = 0.715;
    unloadPoint.target_pose.pose.position.y = 0.2;
    unloadPoint.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14 + 3.14 / 2), unloadPoint.target_pose.pose.orientation);
    success = moveToGoal(unloadPoint);

    moveCommand.linear.x = 1;
    moveCommand.angular.z = 0;
    velPub.publish(moveCommand);
    ros::Duration(0.5).sleep();

    wallFollower(false);
    /*
       // wall follower to exit corridor

       tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14 + 0.4 * 3.14),
                             corridorEntrance.target_pose.pose.orientation);
       success = moveToGoal(corridorEntrance);

       tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14), nearCorridor.target_pose.pose.orientation);
       success = moveToGoal(nearCorridor);

       //fixme maybe one more pos here

       unloadPoint.target_pose.pose.position.x = -1.6;
       unloadPoint.target_pose.pose.position.y = -0.44;
       unloadPoint.target_pose.pose.position.z = 0.0;
       tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14), unloadPoint.target_pose.pose.orientation);
       success = moveToGoal(unloadPoint);

   */    //todo find clear space to start
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
    bool allowNeg = false;

    geometry_msgs::Pose currPose = marrPose;
    while (true) {
        goal.target_pose.header.frame_id = "marrtino_map";
        goal.target_pose.header.stamp = ros::Time::now();

        ROS_INFO("Sending global goal");

        client.sendGoal(goal);
        ros::Duration(2).sleep();

        double diffX = fabs(currPose.position.x - marrPoseOdom.position.x);
        double diffY = fabs(currPose.position.y - marrPoseOdom.position.y);
        if (diffX > +0.05 || diffX < -0.05 ||
            diffY < -0.05 || diffY > +0.05) {
            ROS_INFO_STREAM("Robot has moved somewhere, resetting recovery flags");
            backed = false;
            rotated = false;
            allowNeg = false;
            changeVel(false);
        }


        client.waitForResult();
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
            (fabs(marrPoseOdom.position.x - goal.target_pose.pose.position.x) < 0.2 &&
             fabs(marrPoseOdom.position.y - goal.target_pose.pose.position.y) < 0.2)) {
            ROS_INFO_STREAM("Move successful");
            return true;
        } else {
            if (!backed) {
                ROS_INFO_STREAM("RECOVER MANUAL BACKED");
                recoverManual();
                backed = true;
                currPose = marrPoseOdom;
            } else if (!allowNeg) {
                ROS_INFO_STREAM("RECOVER MANUAL NEGATIVE");
                changeVel(true);
                allowNeg = true;
            } else if (!rotated) {
                ROS_INFO_STREAM("RECOVER MANUAL ROTATION");
                recoverManual(true);
                rotated = true;
                currPose = marrPoseOdom;
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
    double_param.value = negative ? -0.2 : 0.1;
    conf.doubles.push_back(double_param);
    srv_req.config = conf;
    ros::service::call("/marrtino/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

void G01Move::recoverManual(bool rot) {
    //resetting previous moveCommand settings
    moveCommand.linear.x = 0;
    moveCommand.angular.z = 0;
    double defZ = 0.4, defX = 0.2;

    if (rot) {
        //todo refine
        ROS_INFO_STREAM("Rotating..");
        moveCommand.linear.x = defX;
        if (avgSx > avgDx) {
            moveCommand.angular.z = -defZ;
            if (avgDx < 0.3) {
                moveCommand.linear.x = -defX;
                moveCommand.angular.z = defZ;
            }
        } else {
            moveCommand.angular.z = defZ;
            if (avgSx < 0.3) {
                moveCommand.linear.x = -defX;
                moveCommand.angular.z = -defZ;
            }
        }

        velPub.publish(moveCommand);
        ros::Duration(2).sleep();
    } else {
        ROS_INFO_STREAM("Backing up linear");

        tf::Quaternion rotation(marrPoseOdom.orientation.x, marrPoseOdom.orientation.y,
                                marrPoseOdom.orientation.z, marrPoseOdom.orientation.w);
        tf::Vector3 vector(0.25, 0, 0);
        tf::Vector3 rotated_vector = tf::quatRotate(rotation, vector);

        changeVel(true);
        move_base_msgs::MoveBaseGoal goal_temp;
        goal_temp.target_pose.header.frame_id = "marrtino_map";
        goal_temp.target_pose.header.stamp = ros::Time::now();
        goal_temp.target_pose.pose = marrPoseOdom;
        goal_temp.target_pose.pose.position.x -= rotated_vector.getX();
        goal_temp.target_pose.pose.position.y -= rotated_vector.getY();

        MoveBaseClient client_temp("marrtino/move_base", false);
        ROS_INFO("Sending backup goal");
        client_temp.sendGoal(goal_temp);
        client_temp.waitForResult(ros::Duration(2));
        if (client_temp.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            changeVel(false);
            ROS_INFO_STREAM("Backup goal reached");
        }
        client_temp.cancelGoal();
    }
}

void G01Move::wallFollower(bool forward) {
    ROS_INFO_STREAM("WALL START");
    isManualModeDone = false;
    first = true;
    while (!isManualModeDone)
        if (forward)
            followerCallback();
        else
            followerCallback(false);
    ROS_INFO_STREAM("WALL END");
}

void G01Move::rotateDX() {
    ROS_INFO_STREAM("ROTATION START");

    moveCommand.linear.x = 0.2;
    moveCommand.angular.z = -twistVel;
    velPub.publish(moveCommand);
    ros::Duration(1).sleep();

    double r, p, y, ty;
    poseToYPR(marrPoseOdom, &y, &p, &r);
    ty = y;     // current yaw
    ty -= (3.34); // target yaw
    ROS_INFO_STREAM("R " << r << " P " << p << " Y " << y << " TY " << ty);

    // rotate until desired yaw is reached
    moveCommand.linear.x = 0.1;
    moveCommand.angular.z = -1.8 * twistVel;
    while (fabs(y - ty) > 0.1) {
        velPub.publish(moveCommand);
        ros::Duration(0.08).sleep();
        poseToYPR(marrPoseOdom, &y, &p, &r);
    }

    ROS_INFO_STREAM("ROTATION END");
    moveCommand.linear.x = 0.2;
    moveCommand.angular.z = 0;
    velPub.publish(moveCommand);
    ros::Duration(0.8).sleep();
    ROS_INFO_STREAM("LINEAR END");
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

void G01Move::followerCallback(bool forward) {
    ROS_INFO_STREAM("FW " << forwardDist << " DX " << avgDx << " SX " << avgSx << " DIFF " << val);
    // corridor, follow left wall

    double check = (forward ? avgSx : avgDx);

    lateralMinDist = (forward ? 0.28 : 0.26);
    double min_y = (forward ? -2 : -1.2);

    ROS_INFO_STREAM(lateralMinDist);
    ROS_INFO_STREAM(avgDx);
    if (forwardDist > frontWallDist) {
        // assume nearly aligned, we need to move forward

        moveCommand.linear.x = linVel;
        if (check < 0.97 * lateralMinDist) {
            ROS_INFO_STREAM("GO DX");
            moveCommand.angular.z = -1.5 * twistVel;
            moveCommand.linear.x = 0.9 * linVel;
            first = false;
        } else if (check > 1.03 * lateralMinDist) {
            ROS_INFO_STREAM("GO SX");
            moveCommand.angular.z = +1.5 * twistVel;
            if (first) {
                ROS_INFO_STREAM("first");
                moveCommand.angular.z = 2 * twistVel;
                moveCommand.linear.x = 0.5 * linVel;
            }
        } else {
            ROS_INFO_STREAM("AVANTI SAVOIA");
            moveCommand.angular.z = 0.0;
        }
    } else if (marrPoseOdom.position.y < 1.1 && marrPoseOdom.position.y > min_y) {
        // not aligned, just rotate a little
        moveCommand.linear.x = linVel * 1 / 3; // todo maybe unneeded, go with full
        if (check < lateralMinDist) {
            ROS_INFO_STREAM("ALIGN GO DX");
            moveCommand.angular.z = -3 * twistVel;
        } else if (check > 1.1 * lateralMinDist) {
            ROS_INFO_STREAM("ALIGN GO SX");
            moveCommand.angular.z = +3 * twistVel;
        }
    } else {
        // stop
        moveCommand.linear.x = 0.0;
        moveCommand.angular.z = 0.0;
        isManualModeDone = true;
    }
    moveCommand.angular.z = (forward ? moveCommand.angular.z : -moveCommand.angular.z);
    velPub.publish(moveCommand);
    ros::Duration(0.1).sleep();
}

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