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

    // spin a thread by default
    // wait for the action server to come up
    MoveBaseClient client("marrtino/move_base", true);
    while (!client.waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the move_base action server to come up");

    // define poses to reach todo reinsert after tests
    /*nearCorridor.target_pose.pose.position.x = 0.0;
    nearCorridor.target_pose.pose.position.y = -1.5;
    nearCorridor.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, -3.14 / 2), nearCorridor.target_pose.pose.orientation);
    corridorEntrance.target_pose.pose.position.x = 0.55;
    corridorEntrance.target_pose.pose.position.y = -1.4;
    corridorEntrance.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14 / 2), corridorEntrance.target_pose.pose.orientation);
    loadPoint.target_pose.pose.position.x = 0.55;
    loadPoint.target_pose.pose.position.y = 1.6; // dist to go over is 3 mt
    loadPoint.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14 / 2), corridorEntrance.target_pose.pose.orientation);
    unloadPoint.target_pose.pose.position.x = -1.65;
    unloadPoint.target_pose.pose.position.y = -0.4;
    unloadPoint.target_pose.pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 3.14), unloadPoint.target_pose.pose.orientation);*/

    // move to the entrance of the corridor
    //bool success = moveToGoal(client, nearCorridor);
    //success = moveToGoal(client, corridorEntrance);

    // wall follower
    // velocities publisher initialization
    wallFollower(true);

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

bool G01Move::moveToGoal(MoveBaseClient &client, move_base_msgs::MoveBaseGoal goal) {
    // update pose timestamp and send to che robot
    goal.target_pose.header.frame_id = "world";
    goal.target_pose.header.stamp = ros::Time::now();

    ROS_INFO("Sending goal");
    client.sendGoal(goal);

    client.waitForResult();
    return (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
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
    velPub = n.advertise<geometry_msgs::Twist>("/marrtino/cmd_vel", 1000);
    if (forward)
        scannerSub = n.subscribe<sensor_msgs::LaserScan>("/marrtino/scan", 2, &G01Move::forwardCallback, this);
    else
        scannerSub = n.subscribe<sensor_msgs::LaserScan>("/marrtino/scan", 2, &G01Move::backwardCallback, this);
    spinner.start();
}

void G01Move::forwardCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // do not consider movements when not in the right zone
    if(isManualModeDone) return;

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

void G01Move::subPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL) {
    //ROS_INFO_STREAM("Marrtino Pose: " << msgAMCL->pose.pose.position);
    marrPose = msgAMCL->pose.pose;
}