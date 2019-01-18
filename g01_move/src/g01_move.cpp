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

    // move near the corridor area using subsequent goals
    ROS_INFO_STREAM("Go near corridor");
    nearCorridor.target_pose.pose.position.x = 0.15;
    nearCorridor.target_pose.pose.position.y = -1.8;
    nearCorridor.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, PI / 4), nearCorridor.target_pose.pose.orientation);
    success = moveToGoal(nearCorridor);

    //todo tune - too near the entrance and the successive one (y component)
    ROS_INFO_STREAM("Align with corridor entrance");
    corridorEntrance.target_pose.pose.position.x = 0.4;
    corridorEntrance.target_pose.pose.position.y = -1.4;
    corridorEntrance.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 0.4 * PI),
                          corridorEntrance.target_pose.pose.orientation);
    success = moveToGoal(corridorEntrance);


    ROS_INFO_STREAM("Try to go inside corridor");
    corridorInside.target_pose.pose.position.x = 0.65;
    corridorInside.target_pose.pose.position.y = -1.3;
    corridorInside.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 0.4 * PI),
                          corridorInside.target_pose.pose.orientation);
    success = moveToGoal(corridorInside);

    // try to align with the corridor if needed
    alignCorridor();

    wallFollower(true);

    // loading
    ROS_INFO_STREAM("Wait for loading...");
    ros::Duration(2).sleep();

    // rotation
    ROS_INFO_STREAM("Rotate, going back to unload point");
    rotateRight();

    // move to the entrance of the corridor - back
    ROS_INFO_STREAM("Intermediate through the funnel");
    plannerGoal.target_pose.pose.position.x = 0.87;
    plannerGoal.target_pose.pose.position.y = 0.78;
    plannerGoal.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, PI + PI / 3),
                          plannerGoal.target_pose.pose.orientation);
    success = moveToGoal(plannerGoal);


    ROS_INFO_STREAM("Going near the entrance of the corridor");
    plannerGoal.target_pose.pose.position.x = 0.726;
    plannerGoal.target_pose.pose.position.y = 0.2;
    plannerGoal.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, PI + PI / 2),
                          plannerGoal.target_pose.pose.orientation);
    success = moveToGoal(plannerGoal);

    moveCommand.linear.x = 1;
    moveCommand.angular.z = 0;
    velPub.publish(moveCommand);
    ros::Duration(0.5).sleep();

    // wall follower to exit corridor
    wallFollower(false);

    // slightly deviate right after the exit
    deviateRight();

    // back to prev goals
    ROS_INFO_STREAM("Go away from the wall");
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, PI + 0.4 * PI),
                          corridorEntrance.target_pose.pose.orientation);
    success = moveToGoal(corridorEntrance);

    ROS_INFO_STREAM("Align with the arena");
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, PI), nearCorridor.target_pose.pose.orientation);
    success = moveToGoal(nearCorridor);

    ROS_INFO_STREAM("Go to the unload position... Final goal!");
    unloadPoint.target_pose.pose.position.x = -1.6;
    unloadPoint.target_pose.pose.position.y = -0.44;
    unloadPoint.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, PI), unloadPoint.target_pose.pose.orientation);
    success = moveToGoal(unloadPoint);

    //todo rotate and align
    //fixme for real challenge MAYBE find clear space to start - ask
    spinner.stop();
    ros::shutdown(); // todo maybe unneeded, will stop by itself
}

bool G01Move::moveToGoal(move_base_msgs::MoveBaseGoal goal) {
    // wait for the action server to come up
    // MUST leave this to false otherwise spinner will (probably) create conflicts
    MoveBaseClient client("marrtino/move_base", false);

    while (!client.waitForServer(ros::Duration(5.0)))
        ROS_INFO_STREAM("Waiting for the move_base action server to come up");

    bool backed = false;
    bool rotated = false;
    bool allowNeg = false;

    geometry_msgs::Pose currPose = marrPose;
    while (true) {
        goal.target_pose.header.frame_id = "marrtino_map";
        goal.target_pose.header.stamp = ros::Time::now();

        ROS_INFO_STREAM("Sending global goal");

        client.sendGoal(goal);
        ros::Duration(3).sleep();

        double diffX = fabs(currPose.position.x - marrPoseOdom.position.x);
        double diffY = fabs(currPose.position.y - marrPoseOdom.position.y);

        if (diffX > +0.08 || diffX < -0.08 ||
            diffY < -0.08 || diffY > +0.08) {
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
            changeVel(false);
            ROS_INFO_STREAM("Move successful");
            return true;
        } else {
            if (!backed) {
                ROS_WARN_STREAM("Manual recovery: going back");
                recoverManual();
                backed = true;
                currPose = marrPoseOdom;
            } else if (!allowNeg) {
                ROS_WARN_STREAM("Manual recovery: allow negative velocities");
                changeVel(true);
                allowNeg = true;
            } else if (!rotated) {
                ROS_WARN_STREAM("Manual recovery: rotating, dangerous!");
                recoverManual(true);
                rotated = true;
                currPose = marrPoseOdom;
            } else {
                ROS_ERROR_STREAM("Error, robot failed moving");
                changeVel(false);
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
    double defZ = 0.2, defX = 0.2;

    if (rot) {
        //todo refine
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
        //get odom direction
        tf::Quaternion rotation(marrPoseOdom.orientation.x, marrPoseOdom.orientation.y,
                                marrPoseOdom.orientation.z, marrPoseOdom.orientation.w);
        tf::Vector3 vector(0.25, 0, 0);
        //rotate the shift
        tf::Vector3 rotated_vector = tf::quatRotate(rotation, vector);

        changeVel(true);
        move_base_msgs::MoveBaseGoal goal_temp;
        goal_temp.target_pose.header.frame_id = "marrtino_map";
        goal_temp.target_pose.header.stamp = ros::Time::now();
        goal_temp.target_pose.pose = marrPoseOdom;
        //apply the shift
        goal_temp.target_pose.pose.position.x -= rotated_vector.getX();
        goal_temp.target_pose.pose.position.y -= rotated_vector.getY();

        MoveBaseClient client_temp("marrtino/move_base", false);
        client_temp.sendGoal(goal_temp);
        client_temp.waitForResult(ros::Duration(2));
        if (client_temp.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            changeVel(false);
            ROS_INFO_STREAM("Backup goal reached");
        }
        client_temp.cancelGoal();
    }
}

void G01Move::alignCorridor() {
    double r, p, y;
    poseToYPR(marrPoseOdom, &y, &p, &r);

    moveCommand.linear.x = (marrPoseOdom.position.y < -1.35) ? 0.1 : 0.0;

    if (fabs(0.4 * PI - y) > 0.2) {
        if ((0.4 * PI - y) > 0) {
            moveCommand.angular.z = -0.1;
        } else {
            moveCommand.angular.z = 0.1;
        }
    } else
        moveCommand.angular.z = 0.0;
    velPub.publish(moveCommand);
}

void G01Move::rotateRight() {
    // go forward a little
    moveCommand.linear.x = 0.2;
    moveCommand.angular.z = -twistVel;
    velPub.publish(moveCommand);
    ros::Duration(1).sleep();

    double r, p, y, ty;
    poseToYPR(marrPoseOdom, &y, &p, &r);
    ty = y;       // current yaw
    ty -= (3.34); // target yaw

    // rotate until desired yaw is reached
    moveCommand.linear.x = 0.1;
    moveCommand.angular.z = -1.8 * twistVel;
    while (fabs(y - ty) > 0.1) {
        velPub.publish(moveCommand);
        ros::Duration(0.08).sleep();
        poseToYPR(marrPoseOdom, &y, &p, &r);
    }

    // go forward if we are safe
    if (marrPoseOdom.position.x > 0.8) {
        moveCommand.linear.x = 0.2;
        moveCommand.angular.z = 0;
        velPub.publish(moveCommand);
        ros::Duration(0.5).sleep();
    }

    // stop
    moveCommand.linear.x = 0.0;
    moveCommand.angular.z = 0.0;
    velPub.publish(moveCommand);
}

void G01Move::deviateRight() {
    // set right deviation todo tune if needed
    moveCommand.linear.x = 0.2;
    moveCommand.angular.z = -0.1;
    velPub.publish(moveCommand);
    ros::Duration(1).sleep();

    // stop
    moveCommand.linear.x = 0.0;
    moveCommand.angular.z = 0.0;
    velPub.publish(moveCommand);
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

void G01Move::wallFollower(bool forward) {
    isManualModeDone = false;
    first = true;
    while (!isManualModeDone)
        followerCallback(forward);
}

void G01Move::followerCallback(bool forward) {
    //ROS_INFO_STREAM("FW " << forwardDist << " DX " << avgDx << " SX " << avgSx << " = " << val
    //                      << " Y " << marrPoseOdom.position.y);

    if (forwardDist > frontWallDist) {
        // assume nearly aligned, we need to move forward

        moveCommand.linear.x = linVel;
        if (avgSx < 0.98 * lateralMinDist) {
            moveCommand.angular.z = -2 * twistVel;
            moveCommand.linear.x = 0.9 * linVel;
            first = false;
        } else if (avgSx > 1.02 * lateralMinDist) {
            moveCommand.angular.z = +1.5 * twistVel;
            if (first) {
                ROS_INFO_STREAM("First alignment near the wall");
                moveCommand.angular.z = 2 * twistVel;
                moveCommand.linear.x = linVel / 3;
            }
        } else
            moveCommand.angular.z = 0.0;
    } else if ((forward && marrPoseOdom.position.y < 1.1) || (!forward && marrPoseOdom.position.y > -1.2)) {
        // entrance and exit: just rotate a little

        moveCommand.linear.x = linVel * 1 / 3;
        if (avgSx < lateralMinDist)
            moveCommand.angular.z = -3 * twistVel;
        else if (avgSx > 1.1 * lateralMinDist)
            moveCommand.angular.z = +3 * twistVel;
    } else {
        // stop
        moveCommand.linear.x = 0.0;
        moveCommand.angular.z = 0.0;
        isManualModeDone = true;
    }

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