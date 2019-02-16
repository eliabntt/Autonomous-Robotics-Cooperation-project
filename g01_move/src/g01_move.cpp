#include "g01_move.h"

G01Move::G01Move() : n(), spinner(2) {
    ros::NodeHandle nh("~");
    if (nh.hasParam("sim") && (!nh.getParam("sim", sim))) {
        ROS_WARN_STREAM("Invalid value for 'sim' parameter. Setting 'sim' to 'true'.");
        sim = true;
    }

    ROS_INFO_STREAM("Working in " << ((sim) ? "SIMULATION" : "REAL"));
    marrPoseSub = n.subscribe("/marrtino/amcl_pose", 100, &G01Move::subPoseCallback, this);
    if (sim)
        marrPoseOdomSub = n.subscribe("/marrtino/marrtino_base_controller/odom", 100,
                                      &G01Move::subPoseOdomCallback, this);
    else
        marrPoseOdomSub = n.subscribe("/odom", 100, &G01Move::subPoseOdomCallback, this);

    scannerSub = n.subscribe<sensor_msgs::LaserScan>("/marrtino/scan", 2, &G01Move::readLaser, this);
    velPub = n.advertise<geometry_msgs::Twist>("marrtino/move_base/cmd_vel", 10);

    startSub = n.subscribe<std_msgs::Bool>(START_TOPIC, 2, &G01Move::startCallback, this);
    stateSub = n.subscribe<std_msgs::UInt16>(STATE_TOPIC, 2, &G01Move::stateCallback, this);
    statePub = n.advertise<std_msgs::UInt16>(STATE_TOPIC, 2);

    clearMapsClient = n.serviceClient<std_srvs::Empty>("/marrtino/move_base/clear_costmaps");

    spinner.start();

    if (!clearMapsClient.call(empty))
        ROS_INFO_STREAM("Cannot clear the costmaps!");

    // start the loop
    double r, p, y, ty = 0;
    while (anotherRoundNeeded) {
        // wait for proceed command (useful from second run onward)
        while (!proceed)
            ros::Duration(STATE_SLEEP_TIME).sleep();

        // from second run onward, rotate where there are no obstacles, in-place
        if (!firstRun) {
            ROS_INFO_STREAM("Push away from the wall");
            moveCommand.linear.x = -0.1;
            moveCommand.angular.z = 0.0;
            for (int i = 0; i < 20; i++) {
                velPub.publish(moveCommand);
                ros::Duration(0.1).sleep();
            }
            moveCommand.linear.x = 0.0;
            moveCommand.angular.z = 0.0;
            velPub.publish(moveCommand);

            ROS_INFO_STREAM("In-place rotation");
            moveCommand.linear.x = 0.01;
            // set rotation of PI from current yaw (capping result in [-PI,PI])
            ty = (std::min(fabs(y - PI), fabs(y + PI)) == fabs(y - PI)) ? (y - PI) : (y + PI);
            poseToYPR(marrPoseOdom, &y, &p, &r);
            while (fabs(y - ty) > 0.1) {
                //ROS_INFO_STREAM("Y " << y << " TY " << ty);
                moveCommand.angular.z = ((y > ty) ? -0.3 : 0.3);
                velPub.publish(moveCommand);
                ros::Duration(0.08).sleep();
                poseToYPR(marrPoseOdom, &y, &p, &r);
            }
            moveCommand.linear.x = 0.0;
            moveCommand.angular.z = 0.0;
            velPub.publish(moveCommand);

            // re-localization if needed
            if (marrPose.covariance.at(0) > 0.15 || marrPose.covariance.at(7) > 0.15) {
                ros::ServiceClient update_client = n.serviceClient<std_srvs::Empty>(
                        "/marrtino/request_nomotion_update");
                for (int counter = 0; counter < 50; counter++) {
                    update_client.call(empty);
                    ros::Duration(0.01).sleep();
                }
            }
            if (!clearMapsClient.call(empty))
                ROS_INFO_STREAM("Cannot clear the costmaps!");
            ros::Duration(0.1).sleep();
        }
        firstRun = false;

        // publish state (going to load point)
        stateCommand.data = STATE_MARR_RUN;
        statePub.publish(stateCommand);

        // move near the corridor area using subsequent goals
        ROS_INFO_STREAM("Go near corridor");
        changeAnglePrec(true);
        nearCorridor.target_pose.pose.position.x = 0.15;
        nearCorridor.target_pose.pose.position.y = -1.54;
        nearCorridor.target_pose.pose.position.z = 0.0;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, PI / 4 + PI / 2 + PI),
                              nearCorridor.target_pose.pose.orientation);
        success = moveToGoal(nearCorridor);
        changeAnglePrec(false);

        ROS_INFO_STREAM("Align with corridor entrance");
        corridorEntrance.target_pose.pose.position.x = 0.48;
        corridorEntrance.target_pose.pose.position.y = -1.45;
        corridorEntrance.target_pose.pose.position.z = 0.0;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 5 * PI / 12),
                              corridorEntrance.target_pose.pose.orientation);
        success = moveToGoal(corridorEntrance);

        if (!clearMapsClient.call(empty))
            ROS_INFO_STREAM("Cannot clear the costmaps!");

        ROS_INFO_STREAM("Try to go inside corridor");
        corridorInside.target_pose.pose.position.x = 0.55; // little to the left, because planner is crap
        corridorInside.target_pose.pose.position.y = -0.9;
        corridorInside.target_pose.pose.position.z = 0.0;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, PI / 2),
                              corridorInside.target_pose.pose.orientation);
        success = moveToGoal(corridorInside);

        // publish wake up command
        stateCommand.data = STATE_UR10_WAKE;
        statePub.publish(stateCommand);

        ROS_INFO_STREAM("Following the wall");
        wallFollower(true);

        ROS_INFO_STREAM("Docking");
        docking();

        // arrived at loading point: wait for objects detection before
        // issuing the load command and waiting for a response from ur10
        ROS_INFO_STREAM("Waiting for UR10 to be ready...");
        while (currState != STATE_UR10_RDY)
            ros::Duration(STATE_SLEEP_TIME).sleep();

        // ROS_INFO_STREAM("MARR POSE " << marrPoseOdom);
        // ROS_INFO_STREAM("MARR LASER FWD " << forwardDist << " LEFT " << avgSx << " RIGHT " << avgDx);

        stateCommand.data = STATE_UR10_LOAD;
        statePub.publish(stateCommand);
        ROS_INFO_STREAM("Load command issued. Waiting...");

        do { ros::Duration(STATE_SLEEP_TIME).sleep(); } // wait for msg propagation
        while (currState == STATE_UR10_LOAD);

        // save if another round is needed:
        // yes: set to wait for manual restart after unload
        // no:  bypass = set to true
        anotherRoundNeeded = (currState == STATE_UR10_TBC);
        proceed = !anotherRoundNeeded;

        // publish new state (going to unload point)
        stateCommand.data = STATE_MARR_RUN;
        statePub.publish(stateCommand);

        // rotation
        ROS_INFO_STREAM("Rotate, going back to unload point");
        rotateRight(); //fixme is this necessary?

        // move to the entrance of the corridor - back
        ROS_INFO_STREAM("Intermediate through the funnel");
        plannerGoal.target_pose.pose.position.x = 0.65;
        plannerGoal.target_pose.pose.position.y = 0.78;
        plannerGoal.target_pose.pose.position.z = 0.0;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, PI + PI / 3),
                              plannerGoal.target_pose.pose.orientation);
        success = moveToGoal(plannerGoal);

        ROS_INFO_STREAM("Going near the entrance of the corridor");
        plannerGoal.target_pose.pose.position.x = 0.55;
        plannerGoal.target_pose.pose.position.y = 0.25;
        plannerGoal.target_pose.pose.position.z = 0.0;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, PI + PI / 2),
                              plannerGoal.target_pose.pose.orientation);
        success = moveToGoal(plannerGoal);

        ROS_INFO_STREAM("Enter the corridor");
        moveCommand.linear.x = 0.8;
        moveCommand.angular.z = -0.1;
        velPub.publish(moveCommand);
        ros::Duration(0.5).sleep();

        // wall follower to exit corridor
        ROS_INFO_STREAM("Following the wall");
        wallFollower(false);

        // slightly deviate right after the exit
        deviateRight();

        // back to prev goals
        ROS_INFO_STREAM("Align with the arena");
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, PI), nearCorridor.target_pose.pose.orientation);
        success = moveToGoal(nearCorridor);

        ROS_INFO_STREAM("Go to the unload position");
        changeAnglePrec(true);
        unloadPoint.target_pose.pose.position.x = -1.5;
        unloadPoint.target_pose.pose.position.y = -0.4;
        unloadPoint.target_pose.pose.position.z = 0.0;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, PI), unloadPoint.target_pose.pose.orientation);
        success = moveToGoal(unloadPoint);

        // save angle for the second run
        poseToYPR(marrPoseOdom, &y, &p, &r);
        changeAnglePrec(false);

        ROS_INFO_STREAM("Align toward the wall");
        double r, p, y, ty;
        poseToYPR(marrPoseOdom, &y, &p, &r);
        ty = (y > 0) ? PI : -PI; // damn discontinuities
        moveCommand.linear.x = 0.01;
        while (fabs(y - ty) > 0.1) {
            //ROS_INFO_STREAM("Y " << y << " TY " << ty);
            moveCommand.angular.z = (ty > 0) ? 0.3 : -0.3; // works only on half circle
            velPub.publish(moveCommand);
            ros::Duration(0.08).sleep();
            poseToYPR(marrPoseOdom, &y, &p, &r);
        }
        moveCommand.linear.x = 0.0;
        moveCommand.angular.z = 0.0;
        velPub.publish(moveCommand);

        ROS_INFO_STREAM("Advance toward the wall");
        moveCommand.linear.x = 0.4; // todo if tuning needed, here
        moveCommand.angular.z = 0.0;
        velPub.publish(moveCommand);
        ros::Duration(0.5).sleep();
        moveCommand.linear.x = 0.0;
        moveCommand.angular.z = 0.0;
        velPub.publish(moveCommand);

        ROS_INFO_STREAM("Done. Please manually unload objects from marrtino.");
        if (anotherRoundNeeded)
            ROS_INFO_STREAM("Another round is needed. Publish 'true' on /g01_start_run to proceed.");
    }

    spinner.stop();
    ros::shutdown();
}

void G01Move::stateCallback(const std_msgs::UInt16::ConstPtr &msg) {
    // just save current state to the class variable
    currState = msg->data;
}

void G01Move::startCallback(const std_msgs::Bool::ConstPtr &msg) {
    proceed = msg->data;
}

bool G01Move::moveToGoal(move_base_msgs::MoveBaseGoal goal) {
    // wait for the action server to come up
    // MUST leave this to false otherwise spinner will (probably) create conflicts
    MoveBaseClient client("marrtino/move_base", false);

    while (!client.waitForServer(ros::Duration(5.0)))
        ROS_INFO_STREAM("Waiting for the move_base action server to come up");

    bool backed = false;
    bool rotated = false;
    bool cleaned = false;
    int reloc = 0;

    if (marrPose.covariance.at(0) > 0.15 || marrPose.covariance.at(7) > 0.15) {
        ros::ServiceClient update_client = n.serviceClient<std_srvs::Empty>(
                "/marrtino/request_nomotion_update");
        for (int counter = 0; counter < 50; counter++) {
            update_client.call(empty);
            ros::Duration(0.02).sleep();
        }
        reloc += 1;
    }

    geometry_msgs::Pose currPose = marrPoseOdom;

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
            cleaned = false;
            changeVel(false);
            reloc = 0;
        }

        client.waitForResult();
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
            (fabs(marrPoseOdom.position.x - goal.target_pose.pose.position.x) < 0.2 &&
             fabs(marrPoseOdom.position.y - goal.target_pose.pose.position.y) < 0.2)) {
            changeVel(false);
            ROS_INFO_STREAM("Move successful");
            return true;
        } else {
            if (marrPose.covariance.at(0) > 0.2 || marrPose.covariance.at(7) > 0.2) {
                ros::ServiceClient update_client = n.serviceClient<std_srvs::Empty>(
                        "/marrtino/request_nomotion_update");
                for (int counter = 0; counter < 50; counter++) {
                    update_client.call(empty);
                    ros::Duration(0.01).sleep();
                }
                reloc += 1;
                currPose = marrPoseOdom;
            }
            if (reloc > 1 || reloc == 0) {
                if (!backed) {
                    client.cancelAllGoals();
                    client.waitForResult();
                    ROS_WARN_STREAM("Manual recovery: going back");
                    recoverManual();
                    backed = true;
                    currPose = marrPoseOdom;
                } else if (!cleaned) {
                    if (!clearMapsClient.call(empty))
                        ROS_INFO_STREAM("Cannot clear the costmaps!");
                    else {
                        cleaned = true;
                        ros::Duration(0.5).sleep();
                    }
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

                if (marrPose.covariance.at(0) > 0.15 || marrPose.covariance.at(7) > 0.15) {
                    ros::ServiceClient update_client = n.serviceClient<std_srvs::Empty>(
                            "/marrtino/request_nomotion_update");
                    for (int counter = 0; counter < 50; counter++) {
                        update_client.call(empty);
                        ros::Duration(0.01).sleep();
                    }
                    reloc += 1;
                    currPose = marrPoseOdom;
                }
            }
        }
    }
}

void G01Move::changeAnglePrec(bool increase) {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "yaw_goal_tolerance";
    double_param.value = increase ? PI : 0.2;
    conf.doubles.push_back(double_param);
    srv_req.config = conf;
    ros::service::call("/marrtino/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
    ros::Duration(0.2).sleep();
}

void G01Move::changeVel(bool negative) {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "min_vel_x";
    double_param.value = negative ? -0.4 : 0.1;
    conf.doubles.push_back(double_param);
    srv_req.config = conf;
    ros::service::call("/marrtino/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

void G01Move::recoverManual(bool rot) {
    // resetting previous moveCommand settings
    moveCommand.linear.x = 0;
    moveCommand.angular.z = 0;
    double defZ = 0.2, defX = 0.2;

    if (rot) {
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
        // get odom direction
        tf::Quaternion rotation(marrPoseOdom.orientation.x, marrPoseOdom.orientation.y,
                                marrPoseOdom.orientation.z, marrPoseOdom.orientation.w);
        tf::Vector3 vector(0.25, 0, 0);
        // rotate the shift
        tf::Vector3 rotated_vector = tf::quatRotate(rotation, vector);

        changeVel(true);
        move_base_msgs::MoveBaseGoal goal_temp;
        goal_temp.target_pose.header.frame_id = "marrtino_map";
        goal_temp.target_pose.header.stamp = ros::Time::now();
        goal_temp.target_pose.pose = marrPoseOdom;
        // apply the shift
        goal_temp.target_pose.pose.position.x -= rotated_vector.getX();
        goal_temp.target_pose.pose.position.y -= rotated_vector.getY();

        MoveBaseClient client_temp("marrtino/move_base", false);
        client_temp.sendGoal(goal_temp);
        client_temp.waitForResult(ros::Duration(2));
        if (client_temp.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            changeVel(false);
            ROS_INFO_STREAM("Backup goal reached");
        }
        client_temp.cancelAllGoals();
        client_temp.waitForResult();
    }
}

void G01Move::docking() {
    ROS_INFO_STREAM("Rotate toward left wall");
    double r, p, y, ty = PI / 2 + 0.2;
    poseToYPR(marrPoseOdom, &y, &p, &r);
    moveCommand.linear.x = 0.01;
    while (fabs(y - ty) > 0.01) {
        moveCommand.angular.z = ((y > ty) ? -0.3 : 0.3);
        velPub.publish(moveCommand);
        ros::Duration(0.08).sleep();
        poseToYPR(marrPoseOdom, &y, &p, &r);
    }
    moveCommand.linear.x = 0.0; // stop
    moveCommand.angular.z = 0.0;
    velPub.publish(moveCommand);

    ROS_INFO_STREAM("Go on");
    while (marrPoseOdom.position.x > 0.57) {
        //ROS_INFO_STREAM("X " << marrPoseOdom.position.x);
        moveCommand.linear.x = 0.2;
        moveCommand.angular.z = 0.0;
        velPub.publish(moveCommand);
        ros::Duration(0.08).sleep();
    }
    moveCommand.linear.x = 0.0;
    moveCommand.angular.z = 0.0;
    velPub.publish(moveCommand);

    ROS_INFO_STREAM("Realign");
    ty = PI / 2;
    poseToYPR(marrPoseOdom, &y, &p, &r);
    while (fabs(y - ty) > 0.01) {
        //ROS_INFO_STREAM("Y " << y << " TY " << ty);
        moveCommand.angular.z = ((y > ty) ? -0.2 : 0.2);
        velPub.publish(moveCommand);
        ros::Duration(0.08).sleep();
        poseToYPR(marrPoseOdom, &y, &p, &r);
    }
    moveCommand.linear.x = 0.0;
    moveCommand.angular.z = 0.0;
    velPub.publish(moveCommand);

    ROS_INFO_STREAM("Go to loading zone");
    while (forwardDist > 1.15) { // tune
        //ROS_INFO_STREAM("FW " << forwardDist << " Y " << marrPoseOdom.position.y);
        moveCommand.linear.x = 0.2;
        moveCommand.angular.z = 0.0;
        velPub.publish(moveCommand);
        ros::Duration(0.1).sleep();
    }
    moveCommand.linear.x = 0.0;
    moveCommand.angular.z = 0.0;
    velPub.publish(moveCommand);

    ROS_INFO_STREAM("Safety realign");
    poseToYPR(marrPoseOdom, &y, &p, &r);
    while (fabs(y - ty) > 0.01) {
        //ROS_INFO_STREAM("Y " << y << " TY " << ty);
        moveCommand.angular.z = ((y > ty) ? -0.2 : 0.2);
        velPub.publish(moveCommand);
        ros::Duration(0.08).sleep();
        poseToYPR(marrPoseOdom, &y, &p, &r);
    }
    moveCommand.linear.x = 0.0;
    moveCommand.angular.z = 0.0;
    velPub.publish(moveCommand);

    ros::ServiceClient update_client = n.serviceClient<std_srvs::Empty>(
            "/marrtino/request_nomotion_update");
    for (int counter = 0; counter < 50; counter++) {
        update_client.call(empty);
        ros::Duration(0.02).sleep();
    }
}

void G01Move::rotateRight() {
    // go forward a little, pushing away from wall
    double r, p, y, ty;
    poseToYPR(marrPoseOdom, &y, &p, &r);
    ty = y;       // current yaw
    ty -= (3.24); // target yaw

    moveCommand.linear.x = 0.25; //fixme 0.3
    moveCommand.angular.z = -1.2 * twistVel; //fixme 1.3
    velPub.publish(moveCommand);
    ros::Duration(1).sleep();

    // rotate until desired yaw is reached
    moveCommand.linear.x = 0.11;
    moveCommand.angular.z = -1.78 * twistVel;
    while (fabs(y - ty) > 0.05) {
        velPub.publish(moveCommand);
        ros::Duration(0.06).sleep();
        poseToYPR(marrPoseOdom, &y, &p, &r);
    }

    // go forward if we are safe
    if (marrPoseOdom.position.x > 0.7) {
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
    // set right deviation
    moveCommand.linear.x = 0.2;
    moveCommand.angular.z = -0.4;
    velPub.publish(moveCommand);
    ros::Duration(1).sleep();
    moveCommand.linear.x = 0.0;
    moveCommand.angular.z = 0.0;
    velPub.publish(moveCommand);
}

void G01Move::readLaser(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // select values from the whole range
    size = (int) msg->ranges.size();
    avgDx = 0;
    avgSx = 0;

    // scan direction is right to left; start/end is back (0;size)
    readIFront = size / 2;
    readIStart = readIFront - size / 5;
    readIEnd = readIFront + size / 5;

    minDx = msg->ranges[readIStart];
    maxDx = msg->ranges[readIStart];
    minSx = msg->ranges[readIEnd];
    maxSx = msg->ranges[readIEnd];

    // scan and save min avg and max for both sides
    validCount = 0;
    for (int i = readIStart; i < readIStart + howMuchDataToUse; i++) {
        val = msg->ranges[i];
        if (val < readThr) {
            if (val < minDx) minDx = val;
            if (val > maxDx) maxDx = val;
            avgDx += val;
            validCount += 1;
        }
    }
    avgDx /= validCount;
    validCount = 0;
    for (int i = readIEnd; i > readIEnd - howMuchDataToUse; i--) {
        val = msg->ranges[i];
        if (val < readThr) {
            if (val < minSx) minSx = val;
            if (val > maxSx) maxSx = val;
            avgSx += val;
            validCount += 1;
        }
    }
    avgSx /= validCount;
    val = fabs(avgSx - avgDx);

    // distance from front wall
    forwardDist = msg->ranges[readIFront];
}

void G01Move::wallFollower(bool forward) {
    isManualModeDone = false;

    // pose for planner to help in movement
    MoveBaseClient client("marrtino/move_base", false);
    while (!client.waitForServer(ros::Duration(5.0)))
        ROS_INFO_STREAM("Waiting for the move_base action server to come up");

    if (forward) {
        plannerGoal.target_pose.pose.position.x = 0.6;
        plannerGoal.target_pose.pose.position.y = 1.1;
        plannerGoal.target_pose.pose.position.z = 0.0;
        plannerGoal.target_pose.header.frame_id = "marrtino_map";
        plannerGoal.target_pose.header.stamp = ros::Time::now();

        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, PI / 2),
                              plannerGoal.target_pose.pose.orientation);
    } else {
        plannerGoal.target_pose.pose.position.x = 0.5;
        plannerGoal.target_pose.pose.position.y = -1.5;
        plannerGoal.target_pose.pose.position.z = 0.0;
        plannerGoal.target_pose.header.frame_id = "marrtino_map";
        plannerGoal.target_pose.header.stamp = ros::Time::now();

        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, PI * 3 / 2),
                              plannerGoal.target_pose.pose.orientation);
    }
    client.sendGoal(plannerGoal);

    // manual commands
    while (!isManualModeDone)
        followerCallback(forward);

    client.cancelAllGoals();
    client.waitForResult();
}

void G01Move::followerCallback(bool forward) {
    // ROS_INFO_STREAM("FW " << forwardDist << " DX " << avgDx << " SX " << avgSx << " = " << val
    //              << " Y " << marrPoseOdom.position.y);

    // if going forward, stop earlier for docking
    frontWallDist = ((forward) ? 1.6 : 1.15);

    if (forwardDist > frontWallDist) {
        // assume nearly aligned, we need to move forward

        moveCommand.linear.x = linVel;
        if (avgSx < 0.98 * lateralMinDist) {
            //fixme 1.4 1.5
            moveCommand.angular.z = ((!isNearLoadPoint) ? -1.4 * twistVel : -twistVel);
        } else if (avgSx > 1.02 * lateralMinDist) {
            moveCommand.angular.z = ((!isNearLoadPoint) ? +1.4 * twistVel : twistVel);
        } else
            moveCommand.angular.z = 0.0;
    } else if ((forward && marrPoseOdom.position.y < 0.5) || (!forward && marrPoseOdom.position.y > -1.05)) {
        // entrance and exit: just rotate a little

        moveCommand.linear.x = linVel * 1 / 3;
        if (avgSx < lateralMinDist)
            //fixme 2.7 3
            moveCommand.angular.z = -2.7 * twistVel;
        else if (avgSx > 1.1 * lateralMinDist)
            moveCommand.angular.z = +2.7 * twistVel;
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
    marrPose = msgAMCL->pose;
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