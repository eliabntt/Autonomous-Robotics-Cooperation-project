//
// Created by eliabntt on 28/11/18.
//
#include "g01_gripper.h"

std::string planFrameId, endEffId;
bool cylDone = false, triDone = false, cubeDone = false;
geometry_msgs::Pose initialPose;

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
        initialPose = my_group.getCurrentPose().pose;

        subGrab = n.subscribe<g01_perception::PoseStampedArray>("/tags_to_grab", 1000, &G01Gripper::grabCB, this);
        subAvoid = n.subscribe<g01_perception::PoseStampedArray>("/tags_to_avoid", 1000, &G01Gripper::avoidCB, this);

        if (!cubeToGrab.empty() || !cylToGrab.empty() || !triToGrab.empty() || !objectsToAvoid.empty()) {
            subGrab.shutdown();
            subAvoid.shutdown();
            ROS_INFO_STREAM("Tags received");
            finish = true;
        }
    }

    addCollisionWalls(); // surrounding walls

    ROS_INFO_STREAM("Pick and place starting...");

    planning_scene_interface.addCollisionObjects(collision_objects);

    if (!cylToGrab.empty() && !cylDone) {
        cylDone = true;
        moveObjects(my_group, cylToGrab, true); //fixme // with rotation
    }
    if (!cubeToGrab.empty() && !cubeDone) {
        moveObjects(my_group, cubeToGrab);
        cubeDone = true;
    }
    if (!triToGrab.empty() && !triDone) {
        moveObjects(my_group, triToGrab);
        triDone = true;
    }
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
        objectPose.position.z += 0.3;
        objectPose.orientation = initialPose.orientation;

        // compute waypoints on path to the target, create a cartesian path on them
        std::vector<geometry_msgs::Pose> waypoints = makeWaypoints(group.getCurrentPose().pose, objectPose);
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
        double y,p,r;
        double y_ee,p_ee,r_ee;
        poseToYPR(i.pose, &y, &p, &r);
        poseToYPR(pose, &y_ee,&p_ee,&r_ee);
        pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r_ee,p_ee,r-p_ee);

        //todo fixme
        if(i.pose.position.z > 1.06)
            pose.position.z = i.pose.position.z;
        else
            pose.position.z = i.pose.position.z * 1.05;

        move(pose, group);

        // close the gripper, adjust rviz and gazebo
        int how_much = 150;
        gripperClose(how_much);
        ROS_INFO_STREAM("CLOSING");
        while (!isHeld()) {
            how_much += 10;
            gripperClose(how_much);
        }
        group.attachObject(i.header.frame_id, endEffId);
        if (sim) gazeboAttach(linknames[id][0], linknames[id][1]);

        // move to LZ
        // plan and execute the movement
        geometry_msgs::Pose ascent_pose = group.getCurrentPose().pose;
        ascent_pose.position.z += 0.35;
        move(ascent_pose, group);


        geometry_msgs::Pose LZ_pose;
        LZ_pose.position.x = 0.4;
        LZ_pose.position.y = 1.1;
        LZ_pose.position.z = 1.2;

        if (rotate) { //todo check
            LZ_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14 / 2, 0, 0);
            ROS_INFO_STREAM("cylinder will be rotated");
        } else {
            LZ_pose.orientation = initialPose.orientation;
        }

        //todo cylinder get angry here
        move(LZ_pose, group);

        // open the gripper, adjust rviz and gazebo
        group.detachObject(i.header.frame_id); //fixme
        if (sim) gazeboDetach(linknames[id][0], linknames[id][1]);
        gripperOpen();
        removeCollisionBlock(i.header.frame_id);

        ascent_pose = group.getCurrentPose().pose;
        ascent_pose.position.z += 0.35;
        move(ascent_pose, group);
    }
}

void G01Gripper::grabCB(const g01_perception::PoseStampedArray::ConstPtr &input) {
    for (geometry_msgs::PoseStamped item: input->poses) {
        // set position to the center of the object
        std::vector<float> vol = getVolume(item.header.frame_id); // space in x,y,z: tune in tags.h
        ROS_INFO_STREAM((std::find(items.begin(), items.end(), item.header.frame_id) != items.end()));
        if (!(std::find(items.begin(), items.end(), item.header.frame_id) != items.end())) {
            if (vol[0] == vol[2]) {
                cubeToGrab.emplace_back(item);
            } else if (vol[0]*2 == vol[2]) {
                cylToGrab.emplace_back(item);
            } else {
                vol[1] = vol[0];
                vol[2] = vol[0];
                triToGrab.emplace_back(item);
                item.pose.position.z -= vol[2]/2;
                item.pose.position.y += vol[1]/2;
            }
            items.emplace_back(item.header.frame_id);
        }

        item.pose.position.z -= vol[2] / 2;
        collision_objects.emplace_back(addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id));
    }
    ROS_INFO_STREAM("cubes " << cubeToGrab.size() << " cyls " << cylToGrab.size() << " tris " << triToGrab.size());
}

void G01Gripper::avoidCB(const g01_perception::PoseStampedArray::ConstPtr &input) {
    for (geometry_msgs::PoseStamped item: input->poses) {
        if (!(std::find(items.begin(), items.end(), item.header.frame_id) != items.end())) {
            objectsToAvoid.push_back(item);
            items.emplace_back(item.header.frame_id);
        }
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

bool G01Gripper::move(geometry_msgs::Pose destination, moveit::planning_interface::MoveGroupInterface &group) {
    std::vector<geometry_msgs::Pose> waypoints = makeWaypoints(group.getCurrentPose().pose, destination);
    const double JUMP_THRESH = (sim ? 0.0 : 0.1);//fixme no idea if it is a good value
    const double EEF_STEP = 0.01;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESH, trajectory);
    //   ROS_INFO_STREAM("planning result: " << fraction * 100 << "%");
    robot_trajectory::RobotTrajectory robotTraj(group.getCurrentState()->getRobotModel(), PLANNING_GROUP);
    robotTraj.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
    plan.trajectory_ = trajectory;
    moveit_msgs::MoveItErrorCodes resultCode = group.execute(plan);
    //   ROS_INFO_STREAM("movement result: " << resultCode);
    return (resultCode.val == 1);
}

std::vector<geometry_msgs::Pose> G01Gripper::makeWaypoints(geometry_msgs::Pose from, geometry_msgs::Pose to,
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
    //  ROS_INFO_STREAM("makeWaypoints debug: TO_ADD:" << to_add);
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
        /*  ROS_INFO_STREAM("makeWaypoints debug: X:" << intermediate_step.position.x);
          ROS_INFO_STREAM("makeWaypoints debug: Y:" << intermediate_step.position.y);
          ROS_INFO_STREAM("makeWaypoints debug: Z:" << intermediate_step.position.z);
          ROS_INFO_STREAM("makeWaypoints debug: R:" << from_roll + t * to_add[0]);
          ROS_INFO_STREAM("makeWaypoints debug: P:" << from_pitch + t * to_add[1]);
          ROS_INFO_STREAM("makeWaypoints debug: Y:" << from_roll + t * to_add[0]);*/
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

void G01Gripper::addCollisionWalls() {
    geometry_msgs::Pose back_wall;
    back_wall.position.x = 1.2;
    back_wall.position.y = -0.2;
    back_wall.position.z = 1;
    geometry_msgs::Pose side_wall;
    side_wall.position.x = 0.15;
    side_wall.position.y = -0.8;
    side_wall.position.z = 1;
    collision_objects.emplace_back(addCollisionBlock(back_wall, 0.1, 1.3, 2, "back_wall"));
    collision_objects.emplace_back(addCollisionBlock(side_wall, 2, 0.1, 2, "side_wall"));
}
