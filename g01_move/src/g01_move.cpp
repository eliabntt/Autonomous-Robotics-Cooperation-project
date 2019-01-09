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
    scannerSubBis = n.subscribe<sensor_msgs::LaserScan>("/marrtino/scan", 2, &G01Move::averageLR, this);
    velPub = n.advertise<geometry_msgs::Twist>("marrtino/move_base/cmd_vel", 1000);

    nearCorridor.target_pose.pose.position.x = 0.0;
    nearCorridor.target_pose.pose.position.y = -1.8;
    nearCorridor.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 0), nearCorridor.target_pose.pose.orientation);

    moveToGoal(nearCorridor);

    corridorEntrance.target_pose.pose.position.x = 0.6;
    corridorEntrance.target_pose.pose.position.y = -1.1;
    corridorEntrance.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14 / 2), corridorEntrance.target_pose.pose.orientation);
    moveToGoal(corridorEntrance);

    // wall follower
    wallFollower(true);

    /*
    unloadPoint.target_pose.pose.position.x = -1.65;
    unloadPoint.target_pose.pose.position.y = -0.5;
    unloadPoint.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14), unloadPoint.target_pose.pose.orientation);*/

    // rotation
    //move_base_msgs::MoveBaseGoal curpos = corridorEntrance;
    //inPlaceCCW90(curpos);
    //success = moveToGoal(client, curpos);

    // loading

    // rotation and return
    //curpos = loadPoint;
    //inPlaceCCW90(curpos);
    //success = moveToGoal(client, curpos);

    // wall follower
    //wallFollower(false);
    //success = moveToGoal(client,nearCorridor);

    // move to unload position
    //success = moveToGoal(client, unloadPoint);

    // wait for movement to finish
    while (!isManualModeDone) {}
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
              //  padded = false;
            }
            if (!padded) {
            //    changePadding(true);
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
        if (avgL > avgR) {
            moveCommand.angular.z = defZ;
            if (avgR < 0.22) {
                moveCommand.linear.x = -defX;
                moveCommand.angular.z = -defZ;
            }
        } else {
            moveCommand.angular.z = -defZ;
            if (avgL < 0.22) {
                moveCommand.linear.x = -defX;
                moveCommand.angular.z = defZ;
            }
        }

    } else {
        ROS_INFO_STREAM("Backing up linear");
        if (avgL > avgR)
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
    ROS_INFO_STREAM("INTO WALL");
    // initialize topics
    if (forward)
        scannerSub = n.subscribe<sensor_msgs::LaserScan>("/marrtino/scan", 2, &G01Move::forwardCallback, this);
    else
        scannerSub = n.subscribe<sensor_msgs::LaserScan>("/marrtino/scan", 2, &G01Move::backwardCallback, this);
    spinner.start();
}

void G01Move::averageLR(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // do not consider movements when not in the right zone
    if (isManualModeDone) return;

    // select values from the whole range
    size = (int) msg->intensities.size();
    minDx = msg->ranges[0];
    maxDx = msg->ranges[0];
    minSx = msg->ranges[size - 1];
    maxSx = msg->ranges[size - 1];
    avgR = 0;
    avgL = 0;
    for (int i = 0; i < howMuchDataToUse; i++) {
        val = msg->ranges[i];
        if (val < minDx) minDx = val;
        if (val > maxDx) maxDx = val;
        avgR += val;
    }
    for (int i = size - 1; i > size - 1 - howMuchDataToUse; i--) {
        val = msg->ranges[i];
        if (val < minSx) minSx = val;
        if (val > maxSx) maxSx = val;
        avgL += val;
    }

    avgR /= howMuchDataToUse;
    avgL /= howMuchDataToUse;

    // distance from front wall
    forwardDist = msg->ranges[size / 2];

}

void G01Move::forwardCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // do not consider movements when not in the right zone
    if (isManualModeDone) return;

    // select values from the whole range
    size = (int) msg->intensities.size();
    minDx = msg->ranges[0];
    maxDx = msg->ranges[0];
    minSx = msg->ranges[size - 1];
    maxSx = msg->ranges[size - 1];

    // scan and save min and max for both sides
    // scan direction is right to left
    for (int i = 0; i < howMuchDataToUse; i++) {
        val = msg->ranges[i];
        if (val < minDx) minDx = val;
        if (val > maxDx) maxDx = val;
    }
    for (int i = size - 1; i > size - 1 - howMuchDataToUse; i--) {
        val = msg->ranges[i];
        if (val < minSx) minSx = val;
        if (val > maxSx) maxSx = val;
    }

    // distance from front wall
    forwardDist = msg->ranges[size / 2];

    if (forwardDist > loadWallDistance) {
        // we need to move forward
        moveCommand.linear.x = defaultVel;

        val = fabs(minSx - minDx);
        ROS_INFO_STREAM("FW " << forwardDist << " DX " << minDx << " SX " << minSx << " DIFF " << val);
        if ((val > threshMin) && (val < threshMax)) {
            // significant difference but not
            // too high or we are after the corridor
            if (minSx < minDx && minDx > lateralMinDist) {
                // turn right, there is space
                ROS_INFO_STREAM("GO DX");
                moveCommand.angular.z = -0.02;
            } else if (minSx > minDx && minSx > lateralMinDist) {
                // turn left, there is space
                ROS_INFO_STREAM("GO SX");
                moveCommand.angular.z = +0.02;
            } else {
                ROS_INFO_STREAM("AVANTI SAVOIA");
                moveCommand.angular.z = 0.0;
            }
        } else {
            ROS_INFO_STREAM("AVANTI SAVOIA!!!");
            moveCommand.angular.z = 0.0;
        }
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
    // todo maybe useless if we can make it with only the above
    // fixme still not up-to-date like the above version
    int size = (int) msg->intensities.size();
    int howMuch = 5; // angle span to consider
    double val, minSx = msg->ranges[0], maxSx = msg->ranges[0],
            minDx = msg->ranges[size - 1], maxDx = msg->ranges[size - 1];

    // scan and save min and max for both sides
    for (int i = 0; i < howMuch; i++) {
        val = msg->ranges[i];
        if (val < minSx)
            minSx = val;
        if (val > maxSx)
            maxSx = val;
    }
    for (int i = size - 1; i > size - 1 - howMuch; i--) {
        val = msg->ranges[i];
        if (val < minDx)
            minDx = val;
        if (val > maxDx)
            maxDx = val;
    }

    double thresh = 0.0;
    double forwardDist = msg->ranges[size / 2];

    if (forwardDist > corrWallDistance) { // the only thing that changes from above impl.
        // todo also consider range saturation
        // we need to move forward
        moveCommand.linear.x = 0.2;
        if (fabs(minSx - minDx) > thresh) {
            // significant difference
            if (minSx < minDx)
                // turn right
                moveCommand.angular.z = -0.01;
            else
                // turn left
                moveCommand.angular.z = +0.01;
        }
        velPub.publish(moveCommand);
    } else {
        spinner.stop();
        isManualModeDone = true;
    }

    /* ALTERNATIVE WAY
    ROS_INFO("I see: %u", msg->header.seq);
    ROS_INFO("I see: %f", msg->angle_min);
    ROS_INFO("I see: %f", msg->time_increment);

    int size = msg->ranges.size();

    //Variables whith index of highest and lowest value in array.
    int minIndex = 0;
    int maxIndex = 0;

    //This cycle goes through array and finds minimum
    for (int i = minIndex; i < maxIndex; i++)
        if (msg->ranges[link](i) < msg->ranges[link](minIndex) && msg->ranges[link](i) > 0.0)
            minIndex = i;

    //Calculation of angles from indexes and storing data to class variables.
    angleMin = (minIndex - size / 2) * msg->angle_increment;
    double distMin;
    distMin = msg->ranges[link](minIndex);
    distFront = msg->ranges[size / 2];
    diffE = (distMin - wallDistance) - r;
    e = distMin - wallDistance;

    //Invoking method for publishing message
    publishMessage();

    if (false)
        spinner.stop();*/
}

void G01Move::subPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL) {
    marrPose = msgAMCL->pose.pose;
}


void G01Move::subPoseOdomCallback(const nav_msgs::Odometry::ConstPtr &msgOdom) {
    marrPoseOdom = msgOdom->pose.pose;
}