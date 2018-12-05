//
// Created by eliabntt on 28/11/18.
//
#include "g01_gripper.h"

std::string planFrameId, endEffId;

G01Gripper::G01Gripper() : command(), n() {
    //fixme maybe 4 better?
    ros::AsyncSpinner spinner(2);
    spinner.start();

    //todo implement better for wrong inputs(see hw1)
    sim = n.hasParam("sim") ? n.getParam("sim", sim) : true;

    //robot
    moveit::planning_interface::MoveGroupInterface my_group(PLANNING_GROUP);

    //gripper
    gripperCommandPub = n.advertise<robotiq_s_model_control::SModel_robot_output>(
            "/robotiq_hands/l_hand/SModelRobotOutput", 1);
    gripperStatusSub = n.subscribe("/robotiq_hands/l_hand/SModelRobotInput", 1, &G01Gripper::getGripper, this);

    attacher = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    detacher = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");


    //some basic info
    planFrameId = my_group.getPlanningFrame();
    endEffId = my_group.getEndEffectorLink();

    moveit::core::RobotStatePtr current_state = my_group.getCurrentState();

    bool finish = false;
    while (ros::ok() && !finish) {
        gripperOpen();

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;


        my_group.setJointValueTarget(home_joint_positions);

        bool success = (my_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        my_group.move();

        // todo surrounding walls with addCollision
        subGrab = n.subscribe<g01_perception::PoseStampedArray>("/tags_to_grab", 1000, &G01Gripper::grabCB, this);
        subAvoid = n.subscribe<g01_perception::PoseStampedArray>("/tags_to_avoid", 1000, &G01Gripper::avoidCB, this);

        if (!cubeToGrab.empty() || !cylToGrab.empty() || !triToGrab.empty() || !objectsToAvoid.empty()) {
            subGrab.shutdown();
            subAvoid.shutdown();
            ROS_INFO_STREAM("Tags received");
            finish = true;
        }
    }

    ROS_INFO_STREAM("Pick and place starting...");
    //todo add walls
    planning_scene_interface.addCollisionObjects(collision_objects);

    moveObjects(my_group, cylToGrab); //fixme // with rotation
    moveObjects(my_group, cubeToGrab);
    moveObjects(my_group, triToGrab);

    spinner.stop();
    ros::shutdown();
}

void G01Gripper::moveObjects(moveit::planning_interface::MoveGroupInterface &group,
                             std::vector<geometry_msgs::PoseStamped> objectList, bool rotate) {
    if (objectList.empty()) return;

    // settings
    group.setMaxVelocityScalingFactor(0.1);
    group.setGoalPositionTolerance(0.0001);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    long id;
    double y, p, r;
    const double JUMP_THRESH = (sim ? 0.0 : 0.1);//fixme no idea if it is a good value
    const double EEF_STEP = 0.01;
    moveit_msgs::RobotTrajectory trajectory;
    robot_trajectory::RobotTrajectory robotTraj(group.getCurrentState()->getRobotModel(), PLANNING_GROUP);

    // loop through objects
    for (geometry_msgs::PoseStamped i : objectList) {
        id = std::find(tagnames.begin(), tagnames.end(), i.header.frame_id) - tagnames.begin();

        // set target above the object
        geometry_msgs::Pose objectPose;
        objectPose.position = i.pose.position;
        objectPose.position.x += 0.05;
        objectPose.position.z += 0.3;
        poseToYPR(group.getCurrentPose().pose, &y, &p, &r);

        if (rotate) //todo check
            objectPose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14, 3.14, 3.14);
        else
            objectPose.orientation =  group.getCurrentPose().pose.orientation;

        ROS_INFO_STREAM( group.getCurrentPose().pose.orientation);

        // compute waypoints on path to the target, create a cartesian path on them
        std::vector<geometry_msgs::Pose> waypoints = move(group.getCurrentPose().pose, objectPose);
        double fraction = group.computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESH, trajectory);

        // execute the planned movement
        robotTraj.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
        plan.trajectory_ = trajectory;
        moveit_msgs::MoveItErrorCodes resultCode = group.execute(plan); // todo retval unused


        //todo refinment
        double my_x = i.pose.position.x;
        double r_x = group.getCurrentPose().pose.position.x;

        double my_y = i.pose.position.y;
        double r_y = group.getCurrentPose().pose.position.y;

        ROS_INFO_STREAM(my_x << "\n " << r_x << "\n" << my_y << "\n" << r_y);
        while (fabs(my_x - r_x) > 0.03 || fabs(my_y - r_y) > 0.03) {
            geometry_msgs::Pose pose = group.getCurrentPose().pose;
            pose.position.x += my_x - r_x;
            pose.position.y += my_y - r_y;

            group.setPoseTarget(pose);
            bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
                break;
            else
                group.move();
            r_x = group.getCurrentPose().pose.position.x;
            r_y = group.getCurrentPose().pose.position.y;
        }

        //move with the fingers alongside the obj
        geometry_msgs::Pose pose = group.getCurrentPose().pose;
        pose.position.z -= 0.1;
        group.setPoseTarget(pose);
        bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
            group.move();

        // close the gripper, adjust rviz and gazebo
        int how_much = 150;
        gripperClose(how_much);
        ROS_INFO_STREAM("CLOSING");
        while (!isHeld()) {
            how_much += 10;
            gripperClose(how_much);
        }
        // group.attachObject(i.header.frame_id, endEffId); fixme
        if (sim) gazeboAttach(linknames[id][0], linknames[id][1]);

        // back to home todo change this to box LZ
        // plan and execute the movement
        group.setJointValueTarget(home_joint_positions);
        success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
            group.move();
        else
            ROS_ERROR_STREAM("plan to box LZ failed"); // todo build a better error check

        // open the gripper, adjust rviz and gazebo
        // group.detachObject(i.header.frame_id); fixme
        if (sim) gazeboDetach(linknames[id][0], linknames[id][1]);
        gripperOpen();
        removeCollisionBlock(i.header.frame_id);
    }
}

void G01Gripper::grabCB(const g01_perception::PoseStampedArray::ConstPtr &input) {
    for (geometry_msgs::PoseStamped item: input->poses) {
        // set position to the center of the object
        std::vector<float> vol = getVolume(item.header.frame_id); // space in x,y,z: tune in tags.h
        item.pose.position.z -= vol[2] / 2;

        //objectsToGrab.push_back(item);
        if (vol[0] == vol[2])
            cubeToGrab.emplace_back(item);
        else if (vol[0] < vol[2])
            cylToGrab.emplace_back(item);
        else
            triToGrab.emplace_back(item);

        collision_objects.emplace_back(addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id));
    }
    ROS_INFO_STREAM("cubes " << cubeToGrab.size() << " cyls " << cylToGrab.size() << " tris " << triToGrab.size());
}

void G01Gripper::avoidCB(const g01_perception::PoseStampedArray::ConstPtr &input) {
    for (geometry_msgs::PoseStamped item: input->poses) {
        objectsToAvoid.push_back(item);

        std::vector<float> vol = getVolume(item.header.frame_id); // space in x,y,z
        item.pose.position.z -= vol[2] / 2;

        collision_objects.emplace_back(addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id));
    }
}

void G01Gripper::gripperOpen() {
    command.rACT = 1;
    command.rMOD = 0;
    command.rGTO = 1;
    command.rATR = 0;
    command.rGLV = 0;
    command.rICF = 0;
    command.rICS = 0;
    command.rPRA = 0;
    command.rSPA = 0;
    command.rFRA = 0;
    command.rPRB = 0;
    command.rSPB = 0;
    command.rFRB = 255;
    command.rPRC = 0;
    command.rSPC = 0;
    command.rFRC = 0;
    command.rPRS = 0;
    command.rSPS = 0;
    command.rFRS = 0;
    gripperCommandPub.publish(command);
}

//todo probably will need different howMuch based on the object
void G01Gripper::gripperClose(int howMuch) {
    assert(howMuch > 0);
    command.rACT = 1;
    command.rMOD = 0;
    command.rGTO = 1;
    command.rATR = 0;
    command.rGLV = 0;
    command.rICF = 0;
    command.rICS = 0;
    command.rPRA = (unsigned char) howMuch;
    command.rSPA = 255;
    command.rFRA = 150;
    command.rPRB = 0;
    command.rSPB = 0;
    command.rFRB = 0;
    command.rPRC = 0;
    command.rSPC = 0;
    command.rFRC = 0;
    command.rPRS = 0;
    command.rSPS = 0;
    command.rFRS = 0;
    gripperCommandPub.publish(command);
}

moveit_msgs::CollisionObject G01Gripper::addCollisionBlock(geometry_msgs::Pose pose,
                                                           float Xlen, float Ylen, float Zlen, std::string obj_id) {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = planFrameId;
    collision_object.id = obj_id;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = Xlen;
    primitive.dimensions[1] = Ylen;
    primitive.dimensions[2] = Zlen;
    geometry_msgs::Pose box_pose = pose;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    return collision_object;
}

moveit_msgs::CollisionObject G01Gripper::removeCollisionBlock(std::string obj_id) {
    // todo untested
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = planFrameId;
    collision_object.id = obj_id;
    collision_object.operation = collision_object.REMOVE;
    return collision_object;
}


std::vector<geometry_msgs::Pose> G01Gripper::move(geometry_msgs::Pose from, geometry_msgs::Pose to,
                                                  unsigned long n_steps) {
    //divide
    std::vector<geometry_msgs::Pose> steps;
    double t = 0;
    double from_roll, from_pitch, from_yaw;
    double to_roll, to_pitch, to_yaw;
    // double intermediate_roll, intermediate_pitch, intermediate_yaw;
    poseToYPR(from, &from_yaw, &from_pitch, &from_roll);
    poseToYPR(to, &to_yaw, &to_pitch, &to_roll);
    double to_add[] = {(to_roll - from_roll) / n_steps,
                       (to_pitch - from_pitch) / n_steps,
                       (to_yaw - from_yaw) / n_steps};

    for (int idx = 1; idx < n_steps; idx++) {
        t = double(idx) / n_steps;
        geometry_msgs::Pose intermediate_step;
        intermediate_step.position.x = ((1 - t) * from.position.x) + (t * to.position.x);
        intermediate_step.position.y = ((1 - t) * from.position.y) + (t * to.position.y);
        intermediate_step.position.z = ((1 - t) * from.position.z) + (t * to.position.z);
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(from_roll + t * to_add[0],
                                                          from_pitch + t * to_add[1],
                                                          from_yaw + t * to_add[2]),
                              intermediate_step.orientation);

        steps.push_back(intermediate_step);
    }
    steps.push_back(to);

    return steps;
}


void G01Gripper::poseToYPR(geometry_msgs::Pose pose, double *yaw, double *pitch, double *roll) {
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(pose.orientation, quaternion);
    tf::Matrix3x3 mat(quaternion);
    mat.getEulerYPR(*yaw, *pitch, *roll);
}

bool G01Gripper::gazeboAttach(std::string model2, std::string link2) {
    gazebo_ros_link_attacher::AttachRequest req;
    req.model_name_1 = "robot";
    req.link_name_1 = "wrist_3_link";
    req.model_name_2 = model2;
    req.link_name_2 = link2;
    gazebo_ros_link_attacher::AttachResponse res;
    return attacher.call(req, res);
}

bool G01Gripper::gazeboDetach(std::string model2, std::string link2) {
    gazebo_ros_link_attacher::AttachRequest req;
    req.link_name_1 = "wrist_3_link";
    req.link_name_2 = link2;
    req.model_name_1 = "robot";
    req.model_name_2 = model2;
    gazebo_ros_link_attacher::AttachResponse res;
    return detacher.call(req, res);
}


bool G01Gripper::isHeld() {
    //   if (sim) return true;
    ROS_INFO_STREAM("CHECK");
    while (status.gSTA == 0) {
        ROS_INFO_THROTTLE(5, "Waiting grasping to complete...");
    }
    return status.gSTA == 1 || status.gSTA == 2;
}


void G01Gripper::getGripper(const robotiq_s_model_control::SModel_robot_input &msg) {
    status = msg;
}