//
// Created by eliabntt on 28/11/18.
//
#include "g01_gripper.h"

bool cylDone = false, triDone = false, cubeDone = false;
geometry_msgs::Pose initialPose;

G01Gripper::G01Gripper() : command(), n() {
    //fixme maybe 4 better?
    ros::AsyncSpinner spinner(2);
    spinner.start();

    //todo implement better for wrong inputs(see hw1)
    sim = n.hasParam("sim") ? n.getParam("sim", sim) : true;

    // manipulator
    //todo fail-safe if robot_manipulator not found
    moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);

    // gripper
    gripperCommandPub = n.advertise<robotiq_s_model_control::SModel_robot_output>(
            "/robotiq_hands/l_hand/SModelRobotOutput", 1);
    gripperStatusSub = n.subscribe("/robotiq_hands/l_hand/SModelRobotInput", 1, &G01Gripper::gripperCB, this);

    // gazebo fixes
    attacher = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    detacher = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    // some basic info
    planFrameId = group.getPlanningFrame();
    endEffId = group.getEndEffectorLink();

    // move to a home position and wait perception module
    ROS_INFO_STREAM("Waiting to receive tags of objects...");
    bool finish = false;
    while (ros::ok() && !finish) {
        gripperOpen();

        group.setJointValueTarget(HOME_JOINT_POS);
        bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
            group.move();

        // save pose for later (was hardcoded as joints pos)
        initialPose = group.getCurrentPose().pose;

        // subscribe to receive tags poses
        subGrab = n.subscribe<g01_perception::PoseStampedArray>("/tags_to_grab", 1000, &G01Gripper::grabCB, this);
        subAvoid = n.subscribe<g01_perception::PoseStampedArray>("/tags_to_avoid", 1000, &G01Gripper::avoidCB, this);

        if (!cubeToGrab.empty() || !cylToGrab.empty() || !triToGrab.empty() || !objectsToAvoid.empty()) {
            subGrab.shutdown();
            subAvoid.shutdown();
            ROS_INFO_STREAM("Tags received");
            finish = true;
        }
    }

    // add collision objects and surrounding walls to the scene
    addCollisionWalls();
    planning_scene_interface.addCollisionObjects(collision_objects);


    ROS_INFO_STREAM("Pick and place starting...");
    if (!cylToGrab.empty() && !cylDone) {
        cylDone = true;
        moveObjects(group, cylToGrab, true);
    }
    if (!cubeToGrab.empty() && !cubeDone) {
        moveObjects(group, cubeToGrab);
        cubeDone = true;
    }
    if (!triToGrab.empty() && !triDone) {
        moveObjects(group, triToGrab);
        triDone = true;
    }
    spinner.stop();
    ros::shutdown();
}

// Movement
void G01Gripper::moveObjects(moveit::planning_interface::MoveGroupInterface &group,
                             std::vector<geometry_msgs::PoseStamped> objectList, bool rotate) {
    // settings
    group.setMaxVelocityScalingFactor(0.1);
    group.setGoalPositionTolerance(0.0001);

    long id;

    // generic pose and values to be used below
    geometry_msgs::Pose pose;
    double objX, objY, curX, curY;
    double y, p, r, y_ee, p_ee, r_ee;

    // loop through objects
    for (geometry_msgs::PoseStamped i : objectList) {
        id = std::find(tagnames.begin(), tagnames.end(), i.header.frame_id) - tagnames.begin();

        // set target above the object
        geometry_msgs::Pose objectPose;
        objectPose.position = i.pose.position;
        objectPose.position.z += 0.3;
        objectPose.orientation = initialPose.orientation;

        //todo return value to be used for failsafe behaviour
        moveManipulator(objectPose, group);

        // position refinement
        objX = i.pose.position.x;
        objY = i.pose.position.y;
        curX = group.getCurrentPose().pose.position.x;
        curY = group.getCurrentPose().pose.position.y;

        while (fabs(objX - curX) > 0.03 || fabs(objY - curY) > 0.03) {
            pose = group.getCurrentPose().pose;
            pose.position.x += objX - curX;
            pose.position.y += objY - curY;

            group.setPoseTarget(pose);
            bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
                break;
            else
                group.move();
            curX = group.getCurrentPose().pose.position.x;
            curY = group.getCurrentPose().pose.position.y;
        }

        // move with the fingers alongside the obj
        pose = group.getCurrentPose().pose;

        poseToYPR(i.pose, &y, &p, &r);
        poseToYPR(pose, &y_ee, &p_ee, &r_ee);
        pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r_ee, p_ee, fabs(r - p_ee));

        //todo fixme
        if (i.pose.position.z > 1.06)
            pose.position.z = i.pose.position.z;
        else
            pose.position.z = i.pose.position.z * 1.05;

        //todo return value to be used for failsafe behaviour
        moveManipulator(pose, group);

        // close the gripper, adjust rviz and gazebo
        int howMuch = 150;
        gripperClose(howMuch);
        ROS_INFO_STREAM("Closing the gripper");
        while (!isHeld(howMuch)) {
            howMuch += 10;
            gripperClose(howMuch);
        }
        if (sim) gazeboAttach(linknames[id][0], linknames[id][1]);
        group.attachObject(i.header.frame_id, endEffId);

        // go up to safe altitude
        // plan and execute the movement
        pose = group.getCurrentPose().pose;
        pose.position.z += 0.35;
        //todo return value to be used for failsafe behaviour
        moveManipulator(pose, group);

        geometry_msgs::Pose LZ_pose;
        LZ_pose.position.x = 0.4;
        LZ_pose.position.y = 1.1;
        LZ_pose.position.z = pose.position.z;
        LZ_pose.orientation = initialPose.orientation;

        // calculate the new orientation
        if (rotate) { //todo check and refine
            r = +3.14 / 4, p = -3.14 / 4, y = +3.14 / 4;
            tf::Quaternion qRot = tf::createQuaternionFromRPY(r, p, y);
            qRot.normalize(); // todo if general save in H
            tf::Quaternion qFinal = qRot * tf::Quaternion(LZ_pose.orientation.x, LZ_pose.orientation.y,
                                                          LZ_pose.orientation.z, LZ_pose.orientation.w);
            qFinal.normalize();
            tf::quaternionTFToMsg(qFinal, LZ_pose.orientation);
            ROS_INFO_STREAM("Cylinder will be rotated");
        }

        moveManipulator(LZ_pose, group);

        pose = group.getCurrentPose().pose;
        pose.position.z -= 0.4;
        //todo return value to be used for failsafe behaviour
        moveManipulator(pose, group);


        // open the gripper, adjust rviz and gazebo
        group.detachObject(i.header.frame_id);
        if (sim) gazeboDetach(linknames[id][0], linknames[id][1]);
        ROS_INFO_STREAM("Opening the gripper");
        gripperOpen();
        std::vector<std::string> remove = {i.header.frame_id};
        planning_scene_interface.removeCollisionObjects(remove);

        pose = group.getCurrentPose().pose;
        if (rotate) {
            pose.position.x -= 0.4;
            pose.position.y -= 0.4;
            pose.orientation = initialPose.orientation;
        }
        pose.position.z += 0.35;
        //todo return value to be used for failsafe behaviour
        moveManipulator(pose, group);
    }
}

bool
G01Gripper::moveManipulator(geometry_msgs::Pose destination, moveit::planning_interface::MoveGroupInterface &group) {
    // todo comments
    const double JUMP_THRESH = (sim ? 0.0 : 0.1);//fixme no idea if it is a good value
    const double EEF_STEP = 0.01;
    moveit_msgs::RobotTrajectory trajectory;

    double min_threshold = 0.4, success_threshold = 0.8;
    double fraction, best_fraction = 0.0;
    int iteration = 0, max_iterations = 5;
    std::vector<std::vector<geometry_msgs::Pose>> waypoint_trials;
    moveit_msgs::RobotTrajectory trajectory_temp;

    std::vector<geometry_msgs::Pose> waypoints = makeWaypoints(group.getCurrentPose().pose, destination);
    std::vector<unsigned long> steps_vector = {3, 1, 2, 4, 5};
    while (iteration < max_iterations) {
        waypoints = makeWaypoints(group.getCurrentPose().pose, destination, steps_vector[iteration]);
        fraction = group.computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESH, trajectory_temp);
        if (fraction > success_threshold) {
            best_fraction = fraction;
            trajectory = trajectory_temp;
            break;
        } else if (fraction > min_threshold && best_fraction < fraction) {
            trajectory = trajectory_temp;
            best_fraction = fraction;
        }
        iteration++;
    }

    if (best_fraction < min_threshold)
        return false;

    robot_trajectory::RobotTrajectory robotTraj(group.getCurrentState()->getRobotModel(), PLANNING_GROUP);
    robotTraj.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
    plan.trajectory_ = trajectory;
    moveit_msgs::MoveItErrorCodes resultCode = group.execute(plan);
    ROS_INFO_STREAM("Movement result: " << resultCode);
    return (resultCode.val == 1);
}

std::vector<geometry_msgs::Pose> G01Gripper::makeWaypoints(geometry_msgs::Pose from, geometry_msgs::Pose to,
                                                           unsigned long nSteps) {
    // construct intermediate waypoints in path between the two poses
    std::vector<geometry_msgs::Pose> steps;
    double t = 0;
    double Rfrom, Pfrom, Yfrom;
    double Rto, Pto, Yto;
    poseToYPR(from, &Yfrom, &Pfrom, &Rfrom);
    poseToYPR(to, &Yto, &Pto, &Rto);

    // step measures to add
    double step[] = {(Rto - Rfrom) / nSteps,
                     (Pto - Pfrom) / nSteps,
                     (Yto - Yfrom) / nSteps};

    for (int idx = 1; idx < nSteps; idx++) {
        t = double(idx) / nSteps;
        geometry_msgs::Pose intermediate_step;
        intermediate_step.position.x = ((1 - t) * from.position.x) + (t * to.position.x);
        intermediate_step.position.y = ((1 - t) * from.position.y) + (t * to.position.y);
        intermediate_step.position.z = ((1 - t) * from.position.z) + (t * to.position.z);
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(Rfrom + t * step[0],
                                                          Pfrom + t * step[1],
                                                          Yfrom + t * step[2]),
                              intermediate_step.orientation);
        steps.push_back(intermediate_step);
    }
    steps.push_back(to);
    return steps;
}

// Gripper
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

bool G01Gripper::gazeboDetach(std::string name, std::string link) {
    gazebo_ros_link_attacher::AttachRequest req;
    req.model_name_1 = "robot";
    req.link_name_1 = "wrist_3_link";
    req.model_name_2 = name;
    req.link_name_2 = link;
    gazebo_ros_link_attacher::AttachResponse res;
    return detacher.call(req, res);
}

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

bool G01Gripper::gazeboAttach(std::string name, std::string link) {
    gazebo_ros_link_attacher::AttachRequest req;
    req.model_name_1 = "robot";
    req.link_name_1 = "wrist_3_link";
    req.model_name_2 = name;
    req.link_name_2 = link;
    gazebo_ros_link_attacher::AttachResponse res;
    return attacher.call(req, res);
}

// Collisions
void G01Gripper::grabCB(const g01_perception::PoseStampedArray::ConstPtr &input) {
    for (geometry_msgs::PoseStamped item: input->poses) {
        // set position to the center of the object
        std::vector<float> vol = getVolume(item.header.frame_id); // space in x,y,z

        // make sure only to add objects once, split by type
        if (!(std::find(grabObjNames.begin(), grabObjNames.end(), item.header.frame_id) != grabObjNames.end())) {
            if (vol[0] == vol[2]) {
                cubeToGrab.emplace_back(item);
                item.pose.position.z -= vol[2] / 2;
                collision_objects.emplace_back(
                        addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id));
            } else if (vol[0] * 2 == vol[2]) {
                cylToGrab.emplace_back(item);
                item.pose.position.z -= vol[2] / 2;
                collision_objects.emplace_back(
                        addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id));
            } else {
                triToGrab.emplace_back(item);
                collision_objects.emplace_back(
                        addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id, true));
            }
            grabObjNames.emplace_back(item.header.frame_id);
        }
    }
    ROS_INFO_STREAM("cubes " << cubeToGrab.size() << " cyls " << cylToGrab.size() << " tris " << triToGrab.size());
}

void G01Gripper::avoidCB(const g01_perception::PoseStampedArray::ConstPtr &input) {
    for (geometry_msgs::PoseStamped item: input->poses) {
        if (!(std::find(grabObjNames.begin(), grabObjNames.end(), item.header.frame_id) != grabObjNames.end())) {
            objectsToAvoid.push_back(item);
            grabObjNames.emplace_back(item.header.frame_id);
        }
        std::vector<float> vol = getVolume(item.header.frame_id); // space in x,y,z

        if (vol[0] == vol[2] || vol[0] * 2 == vol[2])
            collision_objects.emplace_back(
                    addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id));
        else
            collision_objects.emplace_back(
                    addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id, true));
    }
}

moveit_msgs::CollisionObject G01Gripper::addCollisionBlock(geometry_msgs::Pose pose,
                                                           float Xlen, float Ylen, float Zlen, std::string objectId,
                                                           bool isTriangle) {
    moveit_msgs::CollisionObject co;
    co.header.frame_id = planFrameId;
    co.id = objectId;
    co.operation = co.ADD;

    if (!isTriangle) {
        // add a box-like shape as collision object
        shape_msgs::SolidPrimitive solid;
        solid.type = solid.BOX;
        solid.dimensions.resize(3);
        solid.dimensions[0] = Xlen;
        solid.dimensions[1] = Ylen;
        solid.dimensions[2] = Zlen;
        co.primitives.push_back(solid);
        co.primitive_poses.push_back(pose);
    } else {
        // use a mesh to add a triangle-like shape
        shapes::Mesh *m = shapes::createMeshFromResource("package://challenge_arena/meshes/triangle_centered.stl");

        shape_msgs::Mesh mesh;
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(m, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        co.meshes.resize(1.7); //todo check
        co.mesh_poses.resize(1.7);
        co.meshes[0] = mesh;
        co.mesh_poses[0].position = pose.position;
        co.mesh_poses[0].orientation = pose.orientation;

        double y, p, r;
        poseToYPR(pose, &y, &p, &r);
        r = 0;
        p = 0;

        // rotate the previous pose by 45* about Z because of mesh orientation
        tf::Quaternion qOriginal, qRotation, qFinal;
        qRotation = tf::createQuaternionFromRPY(r, p, y);
        r = 0, p = 0, y = 3.1415 / 2;
        qOriginal = tf::createQuaternionFromRPY(r, p, y);
        qOriginal.normalize();

        // calculate the new orientation
        qFinal = qRotation * qOriginal;
        qFinal.normalize();

        // store the new rotation back into the pose. (requires conversion into a msg type)
        tf::quaternionTFToMsg(qFinal, co.mesh_poses[0].orientation);
        co.meshes.push_back(mesh);
        co.mesh_poses.push_back(co.mesh_poses[0]);
    }
    return co;
}

void G01Gripper::addCollisionWalls() {
    geometry_msgs::Pose backWall;
    backWall.position.x = 1.2;
    backWall.position.y = -0.2;
    backWall.position.z = 1;
    collision_objects.emplace_back(addCollisionBlock(backWall, 0.1, 1.3, 2, "back_wall"));

    geometry_msgs::Pose sideWall;
    sideWall.position.x = 0.15;
    sideWall.position.y = -0.8;
    sideWall.position.z = 1;
    collision_objects.emplace_back(addCollisionBlock(sideWall, 2, 0.1, 2, "side_wall"));
}

// Utilities
void G01Gripper::poseToYPR(geometry_msgs::Pose pose, double *yaw, double *pitch, double *roll) {
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(pose.orientation, quaternion);
    tf::Matrix3x3 mat(quaternion);
    mat.getEulerYPR(*yaw, *pitch, *roll);
}

// fixme
bool G01Gripper::isHeld(int howMuch) {
    // check if object is held by gripper's fingers
    if (sim) return true;

    int precA = -1;
    int precB = -1;
    int precC = -1;
    ros::Duration(1).sleep();;
    // all three in the same position but equal to howMuch
    if ((int) status.gPOA >= howMuch &&
        (int) status.gPOB == (int) status.gPOC &&
        (int) status.gPOB == (int) status.gPOA)
        return false;


    // all fingers in a position != from the previous
    while ((int) status.gPOA != precA && (int) status.gPOB != precB && (int) status.gPOC != precC) {
        precA = status.gPOA;
        precB = status.gPOB;
        precC = status.gPOC;
        ros::Duration(0.2).sleep();
    }
    return true;


    /*
     * alternative method not working on simulation
     *
     * while(status.gSTA == 0) {
     *   ros::Duration(0.2).sleep();
     *   }
     * return status.gSTA == 1 || status.gSTA == 2;
     * if return = false(statu.gSTA == 3) -> chiudi di più
     */
}

void G01Gripper::gripperCB(const robotiq_s_model_control::SModel_robot_input &msg) {
    // just save status of the gripper to be globally used
    status = msg;
}