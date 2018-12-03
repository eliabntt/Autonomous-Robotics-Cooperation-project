//
// Created by eliabntt on 28/11/18.
//
#include "g01_gripper.h"

std::string planFrameId, endEffId;

G01Gripper::G01Gripper() : command(), n() {
    //fixme maybe 4 better?
    ros::AsyncSpinner spinner(2);
    bool finish = false;
    while (n.ok() && !finish) {
        spinner.start();

        //init

        //todo implement better for wrong inputs(see hw1)
        sim = n.hasParam("sim") ? n.getParam("sim", sim) : true;

        //robot
        std::string PLANNING_GROUP = "manipulator";
        moveit::planning_interface::MoveGroupInterface my_group(PLANNING_GROUP);
        //gripper
        gripperCommandPub = n.advertise<robotiq_s_model_control::SModel_robot_output>(
                "/robotiq_hands/l_hand/SModelRobotOutput", 1);


        //some basic info
        planFrameId = my_group.getPlanningFrame();
        endEffId = my_group.getEndEffectorLink();
        ROS_INFO_STREAM("Reference frame: " << planFrameId);
        ROS_INFO_STREAM("End effector frame: " << endEffId);
        geometry_msgs::PoseStamped c = my_group.getCurrentPose(endEffId);
        //ROS_INFO_STREAM(c);


        // todo surrounding walls with addCollision
        subGrab = n.subscribe<g01_perception::PoseStampedArray>("/tags_to_grab", 1000, &G01Gripper::grabCB, this);
        subAvoid = n.subscribe<g01_perception::PoseStampedArray>("/tags_to_avoid", 1000, &G01Gripper::avoidCB, this);


        //todo targe home position to fix the joint links
        geometry_msgs::Pose target_pose1;
        tf::Quaternion q_rot;
        double r = 0, p = 3.145 / 2, y = 0;
        q_rot = tf::createQuaternionFromRPY(r, p, y);
        tf::quaternionTFToMsg(q_rot, target_pose1.orientation);

        target_pose1.position.x = 0.37;
        target_pose1.position.y = -0.12;
        target_pose1.position.z = 1.6;
        my_group.setPoseTarget(target_pose1);

        //todo complete - limit but seems not to work so use the collision https://answers.ros.org/question/273485/move_groupsetworkspace-doesnt-work/
        // my_group.setWorkspace(-1,-1,1,1,1,3);
        my_group.setPoseReferenceFrame("/world");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (my_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        my_group.move();
        ROS_INFO_STREAM("Visualizing plan 1 (pose goal)" << (success ? "" : "FAILED"));

        //gripperClose(255);

        //todo out of the cycle
        planning_scene_interface.addCollisionObjects(collision_objects);

        my_group.move();
        ROS_INFO_STREAM("cacca" << cubeToGrab.empty());
        for (auto i : cubeToGrab) {
            geometry_msgs::Pose test_pose_2 = i.pose;

            test_pose_2.position.z += 0.1;
            double r2 = 3.145 / 2, p2 = 3.145 / 4, y2 = 3.145 / 4;
            //fixme
            // poseToYPR(test_pose_2, &y2, &p2, &r2);
            ROS_INFO_STREAM("ciao");
            q_rot = tf::createQuaternionFromRPY(r2, p2, y2);
            tf::quaternionTFToMsg(q_rot, test_pose_2.orientation);
            move(target_pose1, test_pose_2, my_group);
        }

        finish = false;
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
    ROS_INFO_STREAM("grabCB");
    for (geometry_msgs::PoseStamped item: input->poses) {
        // set position to the center of the object
        std::vector<float> vol = getVolume(item.header.frame_id); // space in x,y,z todo tune in tags.h
        item.pose.position.z -= vol[2] / 2;

        //objectsToGrab.push_back(item);
        ROS_INFO_STREAM(vol[0]<<vol[1]);
        if (vol[0] == vol[2])
            cubeToGrab.emplace_back(item);
        else if (vol[0] < vol[2])
            cylToGrab.emplace_back(item);
        else
            triToGrab.emplace_back(item); // todo evaluate further center/orient tuning, but maybe unneeded

        collision_objects.emplace_back(addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id));
    }
    //ROS_INFO_STREAM("cubes " << cubeToGrab.size() << " cyls " << cylToGrab.size() << " tris " << triToGrab.size());
    ROS_INFO_STREAM("grabCBEnd");
}

void G01Gripper::avoidCB(const g01_perception::PoseStampedArray::ConstPtr &input) {
    ROS_INFO_STREAM("avoidCB");
    for (geometry_msgs::PoseStamped item: input->poses) {
        objectsToAvoid.push_back(item);

        std::vector<float> vol = getVolume(item.header.frame_id); // space in x,y,z
        item.pose.position.z -= vol[2] / 2;

        collision_objects.emplace_back(addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id));
    }
    ROS_INFO_STREAM("avoidCBEnd");
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


void G01Gripper::move(geometry_msgs::Pose from, geometry_msgs::Pose to,
                      moveit::planning_interface::MoveGroupInterface &my_group, unsigned long n_steps) {
    //divide
    std::vector<geometry_msgs::Pose> steps;
    steps.reserve(n_steps);
    double t = 0;
    double from_roll, from_pitch, from_yaw;
    double to_roll, to_pitch, to_yaw;
    double intermediate_roll, intermediate_pitch, intermediate_yaw;
    poseToYPR(from, &from_yaw, &from_pitch, &from_roll);
    poseToYPR(to, &to_yaw, &to_pitch, &to_roll);
    ROS_INFO_STREAM("FROM y: " << from_yaw << " p: " << from_pitch << " r: " << from_yaw);
    ROS_INFO_STREAM("TO y: " << to_yaw << " p: " << to_pitch << " r: " << to_yaw);

    for (int idx = 0; idx < n_steps; idx++) {
        t = double(idx) / n_steps;
        geometry_msgs::Pose intermediate_step;
        intermediate_step.position.x = ((1 - t) * from.position.x) + (t * to.position.x);
        intermediate_step.position.y = ((1 - t) * from.position.y) + (t * to.position.y);
        intermediate_step.position.z = ((1 - t) * from.position.z) + (t * to.position.z);
        intermediate_roll = ((1 - t) * from_roll) + (t * to_roll);
        intermediate_pitch = ((1 - t) * from_pitch) + (t * to_pitch);
        intermediate_yaw = ((1 - t) * from_yaw) + (t * to_yaw);
        ROS_INFO_STREAM("t: " << t);
        ROS_INFO_STREAM("y: " << intermediate_yaw << " p: " << intermediate_pitch << " r: " << intermediate_yaw);
        ROS_INFO_STREAM("x: " << intermediate_step.position.x);
        intermediate_step.orientation = tf::createQuaternionMsgFromRollPitchYaw(intermediate_roll, intermediate_pitch,
                                                                                intermediate_yaw);
        steps.emplace_back(intermediate_step);
    }
    steps.emplace_back(to);

    //plan
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = (sim ? 0.0 : 0.1);//fixme no idea if it is a good value
    const double eef_step = 0.01;
    double success = 0;
    for (int i = 0; i < 3; i++) {
        success = my_group.computeCartesianPath(steps, eef_step, jump_threshold, trajectory);
        ROS_INFO_STREAM("movement planning: " << success * 100.0 << "% achieved");
        my_group.move();
        break;
    }
}


void G01Gripper::poseToYPR(geometry_msgs::Pose pose, double *yaw, double *pitch, double *roll) {
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(pose.orientation, quaternion);
    tf::Matrix3x3 mat(quaternion);
    mat.getEulerYPR(*yaw, *pitch, *roll);
}