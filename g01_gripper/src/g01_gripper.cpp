//
// Created by eliabntt on 28/11/18.
//
#include "g01_gripper.h"

std::string planFrameId, endEffId;

G01Gripper::G01Gripper() : command(), n() {
    //fixme maybe 4 better?
    ros::AsyncSpinner spinner(0);
    bool finish = false;
    spinner.start();

    while (ros::ok() && !finish) {
        //init

        //todo implement better for wrong inputs(see hw1)
        sim = n.hasParam("sim") ? n.getParam("sim", sim) : true;

        //robot
        std::string PLANNING_GROUP = "manipulator";
        moveit::planning_interface::MoveGroupInterface my_group(PLANNING_GROUP);
        my_group.setPlannerId("RRTConnectkConfigDefault");
        ROS_INFO_STREAM(my_group.getDefaultPlannerId());
        //gripper
        gripperCommandPub = n.advertise<robotiq_s_model_control::SModel_robot_output>(
                "/robotiq_hands/l_hand/SModelRobotOutput", 1);

        const robot_state::JointModelGroup *joint_model_group =
                my_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        //some basic info
        planFrameId = my_group.getPlanningFrame();
        endEffId = my_group.getEndEffectorLink();

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        moveit::core::RobotStatePtr current_state = my_group.getCurrentState();
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group,
                                               joint_group_positions);
        joint_group_positions[0] = -3.14/2; // radians
        joint_group_positions[1] = -1.86; // radians
        joint_group_positions[2] = 1.72788; // radians
        joint_group_positions[3] = -1.65; // radians
        joint_group_positions[4] = -3.14/2; // radians
        joint_group_positions[5] = 3.14/3; // radians
        my_group.setJointValueTarget(joint_group_positions);

        bool success = (my_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        my_group.move();

        // todo surrounding walls with addCollision
        subGrab = n.subscribe<g01_perception::PoseStampedArray>("/tags_to_grab", 1000, &G01Gripper::grabCB, this);
        subAvoid = n.subscribe<g01_perception::PoseStampedArray>("/tags_to_avoid", 1000, &G01Gripper::avoidCB, this);

        //gripperClose(255);

        //todo out of the cycle
        planning_scene_interface.addCollisionObjects(collision_objects);

        for (auto i : cubeToGrab) {

            geometry_msgs::Pose test_pose_2;
            test_pose_2.position = i.pose.position;
            test_pose_2.position.z += 0.8;
            test_pose_2.orientation = my_group.getCurrentPose("ee_link").pose.orientation;

            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = (sim ? 0.0 : 0.1);//fixme no idea if it is a good value
            const double eef_step = 0.01;
            std::vector<geometry_msgs::Pose> waypoints = move(my_group.getCurrentPose().pose, test_pose_2, my_group);

            my_group.setMaxVelocityScalingFactor(0.1);

            double fraction = my_group.computeCartesianPath(waypoints, eef_step,
                                                            jump_threshold, trajectory);

            ROS_INFO_STREAM("che cazzo " << fraction << i.header);
            robot_trajectory::RobotTrajectory rt(my_group.getCurrentState()->getRobotModel(),
                                                 PLANNING_GROUP);
            rt.setRobotTrajectoryMsg(*my_group.getCurrentState(), trajectory);

            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            bool time_success = iptp.computeTimeStamps(rt);
            rt.getRobotTrajectoryMsg(trajectory);
            my_plan.trajectory_ = trajectory;
            moveit_msgs::MoveItErrorCodes a = my_group.execute(my_plan);
            ROS_INFO_STREAM(a);
            finish = true;
        }
    }
    spinner.stop();


/*
    open();
    if(sim)
        my_group.detachObject(id);

    close(255);
    if(sim)
        my_group.attatchObject(id, endEffId);
 */
}

void G01Gripper::grabCB(const g01_perception::PoseStampedArray::ConstPtr &input) {
    if (cubeToGrab.empty() && cylToGrab.empty() && triToGrab.empty() && objectsToAvoid.empty())
        for (geometry_msgs::PoseStamped item: input->poses) {
            // set position to the center of the object
            std::vector<float> vol = getVolume(item.header.frame_id); // space in x,y,z todo tune in tags.h
            item.pose.position.z -= vol[2] / 2;

            //objectsToGrab.push_back(item);
            if (vol[0] == vol[2])
                cubeToGrab.emplace_back(item);
            else if (vol[0] < vol[2])
                cylToGrab.emplace_back(item);
            else
                triToGrab.emplace_back(item); // todo evaluate further center/orient tuning, but maybe unneeded

            collision_objects.emplace_back(addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id));
            ROS_INFO_STREAM("cubes " << cubeToGrab.size() << " cyls " << cylToGrab.size() << " tris " << triToGrab.size());
        }
    else
        subGrab.shutdown();
}

void G01Gripper::avoidCB(const g01_perception::PoseStampedArray::ConstPtr &input) {
    if (cubeToGrab.empty() && cylToGrab.empty() && triToGrab.empty() && objectsToAvoid.empty())
        for (geometry_msgs::PoseStamped item: input->poses) {
            objectsToAvoid.push_back(item);

            std::vector<float> vol = getVolume(item.header.frame_id); // space in x,y,z
            item.pose.position.z -= vol[2] / 2;

            collision_objects.emplace_back(addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id));
        }
    else
        subAvoid.shutdown();
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
                                                  moveit::planning_interface::MoveGroupInterface &my_group,
                                                  unsigned long n_steps) {
    //divide
    std::vector<geometry_msgs::Pose> steps;
    double t = 0;
    // double from_roll, from_pitch, from_yaw;
    // double to_roll, to_pitch, to_yaw;
    // double intermediate_roll, intermediate_pitch, intermediate_yaw;
    // poseToYPR(from, &from_yaw, &from_pitch, &from_roll);
    // poseToYPR(to, &to_yaw, &to_pitch, &to_roll);
    // ROS_INFO_STREAM("FROM y: " << from_yaw << " p: " << from_pitch << " r: " << from_yaw);
    //  ROS_INFO_STREAM("TO y: " << to_yaw << " p: " << to_pitch << " r: " << to_yaw);

    for (int idx = 0; idx < n_steps; idx++) {
        t = double(idx) / n_steps;
        geometry_msgs::Pose intermediate_step;
        intermediate_step.position.x = ((1 - t) * from.position.x) + (t * to.position.x);
        ROS_INFO_STREAM(intermediate_step.position.x);

        intermediate_step.position.y = ((1 - t) * from.position.y) + (t * to.position.y);
        intermediate_step.position.z = ((1 - t) * from.position.z) + (t * to.position.z);
    //  intermediate_roll = ((1 - t) * from_roll) + (t * to_roll);
    //  intermediate_pitch = ((1 - t) * from_pitch) + (t * to_pitch);
    //  intermediate_yaw = ((1 - t) * from_yaw) + (t * to_yaw);
    //  ROS_INFO_STREAM("x: " << intermediate_step.position.x);
    //  intermediate_step.orientation = tf::createQuaternionMsgFromRollPitchYaw(intermediate_roll, intermediate_pitch,

        intermediate_step.orientation = from.orientation;
        steps.push_back(intermediate_step);
    }
    ROS_INFO_STREAM(to.position.x);
    steps.push_back(to);

    return steps;
}


void G01Gripper::poseToYPR(geometry_msgs::Pose pose, double *yaw, double *pitch, double *roll) {
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(pose.orientation, quaternion);
    tf::Matrix3x3 mat(quaternion);
    mat.getEulerYPR(*yaw, *pitch, *roll);
}