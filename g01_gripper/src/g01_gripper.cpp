//
// Created by eliabntt on 28/11/18.
//
#include "g01_gripper.h"

G01Gripper::G01Gripper() : command(), n() {
    //fixme maybe 4 better
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle nh("~");
    if (nh.hasParam("sim") && (!nh.getParam("sim", sim))) {
        ROS_WARN_STREAM("Invalid value for 'sim' parameter. Setting 'sim' to 'true'.");
        sim = true;
    }
    ROS_INFO_STREAM("Working in " << ((sim) ? "SIMULATION" : "REAL"));

    // state
    stateSub = n.subscribe<std_msgs::UInt16>(STATE_TOPIC, 2, &G01Gripper::stateCallback, this);
    statePub = n.advertise<std_msgs::UInt16>(STATE_TOPIC, 2);

    // manipulator
    moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);

    // gripper
    gripperCommandPub = n.advertise<robotiq_s_model_control::SModel_robot_output>(
            "/robotiq_hands/l_hand/SModelRobotOutput", 1);
    fakeGripperCommandPub = n.advertise<std_msgs::UInt16>("/angle_motor", 1);
    gripperStatusSub = n.subscribe("/robotiq_hands/l_hand/SModelRobotInput", 1, &G01Gripper::gripperCB, this);

    // gazebo fixes
    attacher = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    detacher = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    // some basic info
    planFrameId = group.getPlanningFrame();
    endEffId = group.getEndEffectorLink();

    // SAFETY FIRST! add surrounding walls to the scene only in real scenario
    // (assume manipulator is somewhere over the table, while in sim it starts horizontal
    // and it traverses the walls so that planner constantly fails)
    if (!sim) {
        addCollisionWalls();
        planningSceneIF.addCollisionObjects(collObjects);
    }

    // start the loop
    while (!finish) {
        if (currState == STATE_UR10_WAKE) {
            if (tagsReceived) continue;

            // marrtino is approaching the corridor: start object detection
            ROS_INFO_STREAM("Waiting to receive tags of objects...");

            // move to a home position
            goHome(group);

            // save pose for later (was hardcoded as joints angles)
            initialPose = group.getCurrentPose().pose;

            // subscribe to receive tags poses
            subGrab = n.subscribe<g01_perception::PoseStampedArray>("/g01_tags_grab", 10, &G01Gripper::grabCB, this);
            subAvoid = n.subscribe<g01_perception::PoseStampedArray>("/g01_tags_avoid", 10, &G01Gripper::avoidCB, this);

            // wait for perception module
            bool stop = false;
            while (ros::ok() && !stop)
                if (!cubeToGrab.empty() || !cylToGrab.empty() || !triToGrab.empty() || !objectsToAvoid.empty()) {
                    // objects detected, proceed to next step
                    subGrab.shutdown();
                    subAvoid.shutdown();
                    stop = true;
                    ROS_INFO_STREAM("Tags received");
                }

            // add objects as collision items (elaborated in callbacks)
            // also add walls if in simulation (manipulator already moved to home)
            if (sim) addCollisionWalls();
            planningSceneIF.addCollisionObjects(collObjects);

            // send ready state signal
            stateCommand.data = STATE_UR10_RDY;
            statePub.publish(stateCommand);
            tagsReceived = true;

        } else if (currState == STATE_UR10_LOAD) {
            if (!tagsReceived) continue;

            // wait for marrtino pose and transform to come up
            if (sim)
                marrOdomSub = n.subscribe("/marrtino/marrtino_base_controller/odom", 1,
                                          &G01Gripper::marrOdomCallback, this);
            else
                marrOdomSub = n.subscribe("/odom", 1, &G01Gripper::marrOdomCallback, this);

            // strategy: if planning fails objects are placed in the return vector;
            // retry the call for max 5 times if needed

            if (odomReceived) {
                // marrtino is in position: move objects
                ROS_INFO_STREAM("Pick and place starting...");

                ObjectBox box = ObjectBox(LZPose);
                finish = true; // can change later
                full = false;  // empty box

                // move cylinders (hexagons)
                int count = 0;
                while (!cylToGrab.empty() && count < 5 && !full) {
                    cylToGrab = moveObjects(group, cylToGrab, box, true);
                    count += 1;
                }
                if (!cylToGrab.empty()) {
                    ROS_ERROR_STREAM("Error, " << cylToGrab.size() << " Hexagon(s) cannot be placed");
                    finish = false;
                }

                // move cubes
                count = 0;
                while (!cubeToGrab.empty() && count < 5 && !full) {
                    cubeToGrab = moveObjects(group, cubeToGrab, box);
                    count += 1;
                }
                if (!cubeToGrab.empty()) {
                    ROS_ERROR_STREAM("Error, " << cubeToGrab.size() << " Cube(s) cannot be placed");
                    finish = false;
                }

                // move prisms
                count = 0;
                while (!triToGrab.empty() && count < 5 && !full) {
                    triToGrab = moveObjects(group, triToGrab, box);
                    count += 1;
                }
                if (!triToGrab.empty()) {
                    ROS_ERROR_STREAM("Error, " << triToGrab.size() << " Trapezoid(s) cannot be placed");
                    finish = false;
                }

                // send new state signal: finish is true only if all the three lists are empty
                stateCommand.data = (finish ? STATE_UR10_DONE : STATE_UR10_TBC);
                statePub.publish(stateCommand);
                tagsReceived = false;

                goHome(group);
            }
        } else ros::Duration(STATE_SLEEP_TIME).sleep();
    }
}

void G01Gripper::stateCallback(const std_msgs::UInt16::ConstPtr &msg) {
    // just save current state to the class variable
    currState = msg->data;
}

// Movement
std::vector<geometry_msgs::PoseStamped> G01Gripper::moveObjects(moveit::planning_interface::MoveGroupInterface &group,
                                                                std::vector<geometry_msgs::PoseStamped> objectList,
                                                                ObjectBox &box, bool rotate) {
    // settings //fixme with real ones
    group.setMaxVelocityScalingFactor(0.9);
    group.setGoalPositionTolerance(0.0001);

    // vector of objects for which the planning failed
    std::vector<geometry_msgs::PoseStamped> remaining;

    // generic poses and values to be used below
    geometry_msgs::Pose pose;
    double objX, objY, curX, curY;
    double y, p, r, y_ee, p_ee, r_ee;
    long index;

    // just to be sure
    gripperOpen();

    // loop through objects
    for (geometry_msgs::PoseStamped obj : objectList) {
        // first thing check if there is available space
        // retrieve destination point
        bool canPlace;
        geometry_msgs::Pose destPose;
        if (rotate) // bad hack to distinguish cylinders from cubes _and_ triangles fixme use better check
            canPlace = box.getCylinderPose(destPose);
        else
            canPlace = box.getCubePose(destPose);
        if (!canPlace) {
            ROS_ERROR_STREAM("Cannot move to box, no place to put object");
            remaining.emplace_back(obj);
            full = true;
            break;
        }

        // get index of object's name to extract Gazebo's object and link names
        index = std::find(tagnames.begin(), tagnames.end(), obj.header.frame_id) - tagnames.begin();

        // set target above the object
        pose.position = obj.pose.position;
        pose.position.z += 0.4;

        poseToYPR(obj.pose, &y, &p, &r);
        poseToYPR(group.getCurrentPose().pose, &y_ee, &p_ee, &r_ee);
        double diff, diffAbs;

        diffAbs = std::min(fabs(y + p_ee), fabs(y - p_ee));
        //todo test this if
        if (diffAbs - PI / 2 > 0.1 || diffAbs > 0.1) {
            if (fabs(y + p_ee) == diffAbs)
                diff = y + p_ee;
            else
                diff = y - p_ee;
        }

        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(r_ee, p_ee, y_ee) * tf::createQuaternionFromRPY(-diff, 0, 0),
                              pose.orientation);

        // move or go back home
        if (!moveManipulator(pose, group)) {
            goHome(group);

            remaining.emplace_back(obj);
            continue; // next object
        }

        // get the right altitude for gripper:
        // anti squish countermeasure
        if (obj.pose.position.z > 1.2)
            pose.position.z = obj.pose.position.z; // cylinders
        else
            pose.position.z = obj.pose.position.z * 1.12;

        // move or go back home (then save new position of the object:
        // movement can stop anywhere between start and stop positions)
        if (!moveManipulator(pose, group)) {
            goHome(group);
            remaining.emplace_back(obj);
            continue;
        }

        // close the gripper, adjust rviz and gazebo:
        // go on closing until gripper feedback assure correct grasp
        int howMuch = 100;
        gripperClose(howMuch);
        ROS_INFO_STREAM("Closing the gripper");
        while (!isHeld(howMuch)) {
            howMuch += 10;
            gripperClose(howMuch);
        }
        if (sim) gazeboAttach(linknames[index][0], linknames[index][1]);
        group.attachObject(obj.header.frame_id, endEffId);

        // go up to a safe altitude
        pose = group.getCurrentPose().pose;
        pose.position.z += 0.35;

        // move or open the gripper and go home:
        // safe to open because still near the table
        if (!moveManipulator(pose, group)) {
            // detach object
            gripperOpen();
            group.detachObject(obj.header.frame_id);
            if (sim) gazeboDetach(linknames[index][0], linknames[index][1]);

            int z = obj.pose.position.z;
            obj.pose.position = group.getCurrentPose().pose.position;
            obj.pose.position.z = z;

            goHome(group);

            // save new object position
            remaining.emplace_back(obj);
            continue;
        }

        // fixme safe altitude
        pose = destPose;
        pose.position.z += 0.6;
        pose.orientation = initialPose.orientation;
        // calculate the new orientation of the "wrist"
        if (rotate) {//todo check if position is right
            ROS_INFO_STREAM("Cylinder will be tilted");
            tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, -PI / 3, 0) *
                                  tf::Quaternion(pose.orientation.x, pose.orientation.y,
                                                 pose.orientation.z, pose.orientation.w), pose.orientation);
            pose.position.x -= 0.15;//fixme
        }

        // add offsets due to real map inconsistencies fixme remove when map will be adjusted
        if (!sim) {
            pose.position.x += 0.32;
            pose.position.y += 0.15;
        }

        if (!moveManipulator(pose, group)) {
            if (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                group.move();
            else {
                // I do not know where I am
                // try to go back down (less to be in a safe position)
                ROS_INFO_STREAM("Error trying to get into placement position");
                pose = group.getCurrentPose().pose;

                pose.position.z -= 0.1;
                moveManipulator(pose, group);

                // detach object
                group.detachObject(obj.header.frame_id);
                if (sim) gazeboDetach(linknames[index][0], linknames[index][1]);

                goHome(group);

                std::vector<std::string> toRemove = {obj.header.frame_id};
                planningSceneIF.removeCollisionObjects(toRemove);
                continue;
            }
        } else {
            ROS_INFO_STREAM("movement to placement position completed");
        }

        // position refinement (another plan step, just to be sure)
        objX = pose.position.x;
        objY = pose.position.y;
        curX = group.getCurrentPose().pose.position.x;
        curY = group.getCurrentPose().pose.position.y;
        while (fabs(objX - curX) > 0.03 || fabs(objY - curY) > 0.03) {
            pose = group.getCurrentPose().pose;
            pose.position.x += objX - curX;
            pose.position.y += objY - curY;

            group.setPoseTarget(pose);
            if (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                group.move();
            else break;
            curX = group.getCurrentPose().pose.position.x;
            curY = group.getCurrentPose().pose.position.y;
        }

        // approach the LZ from above
        pose = group.getCurrentPose().pose;
        pose.position.z -= 0.2;
        if (!rotate) pose.position.z -= 0.05;

        // let the piece fall or go home:
        // no need to set it as remaining (already over the LZ and I cannot go down)
        if (!moveManipulator(pose, group)) {
            group.detachObject(obj.header.frame_id);
            if (sim) gazeboDetach(linknames[index][0], linknames[index][1]);
            goHome(group);
            continue;
        } else
            ROS_INFO_STREAM("Arm lowering completed");

        // open the gripper, adjust rviz and gazebo, mark occupied space in the box
        group.detachObject(obj.header.frame_id);
        if (sim) gazeboDetach(linknames[index][0], linknames[index][1]);
        ROS_INFO_STREAM("Opening the gripper");
        gripperOpen();
        if (rotate)
            box.markCylinderOcc(destPose);
        else
            box.markCubeOcc(destPose);

        // remove the corresponding collision object from the scene
        std::vector<std::string> toRemove = {obj.header.frame_id};
        planningSceneIF.removeCollisionObjects(toRemove);

        pose = group.getCurrentPose().pose;
        pose.position.z += 0.4;
        pose.orientation = initialPose.orientation;

        // here just need to go home
        if (!moveManipulator(pose, group)) {
            goHome(group);
            continue;
        }
    }

    return remaining;
}

bool G01Gripper::moveManipulator(geometry_msgs::Pose destination,
                                 moveit::planning_interface::MoveGroupInterface &group) {
    // cartesian path parameters
    const double JUMP_THRESH = 0.0; // fixme check if good values
    const double EEF_STEP = 0.01;
    moveit_msgs::RobotTrajectory traj, trajTemp;

    // planning trials parameters (thresholds and temp)
    std::vector<unsigned long> stepsVector = {3, 1, 2, 4, 5};
    double minThr = 0.4, succThr = 0.8;
    double fraction, bestFraction = 0.0;

    // if the planning is better than succThr then execute the movement
    // if the planning is better than minThr retry with another number of intermediate steps
    // if the planning does not finally succeed return false
    for (unsigned long steps:stepsVector) {
        moveit_msgs::MoveItErrorCodes errorCode;
        fraction = group.computeCartesianPath(makeWaypoints(group.getCurrentPose().pose, destination, steps),
                                              EEF_STEP, JUMP_THRESH, trajTemp, true, &errorCode);
        if (errorCode.val != 1)
            ROS_ERROR_STREAM("MOVEIT_ERROR_CODE:" << errorCode);
        if (fraction >= succThr) {
            // very good plan
            bestFraction = fraction;
            traj = trajTemp;
            break;
        } else if (fraction > minThr && bestFraction < fraction) {
            // acceptable and better than before
            bestFraction = fraction;
            traj = trajTemp;
        }
    }
    //ROS_INFO_STREAM("Best_Fraction: " << bestFraction);
    // bad plan with all possible steps number, exit
    if (bestFraction < minThr) {
        ROS_INFO_STREAM("PLANNING FAILED: result [" << bestFraction << "] under minimal threshold");
        return false;
    }

    // set the trajectory and execute
    robot_trajectory::RobotTrajectory robotTraj(group.getCurrentState()->getRobotModel(), PLANNING_GROUP);
    robotTraj.setRobotTrajectoryMsg(*group.getCurrentState(), traj);
    plan.trajectory_ = traj;

    moveit_msgs::MoveItErrorCodes resultCode = group.execute(plan);
    return (resultCode.val == 1);
    // return true only if the movement is correct (most times: bad planning removed before)
}

std::vector<geometry_msgs::Pose> G01Gripper::makeWaypoints(geometry_msgs::Pose from, geometry_msgs::Pose to,
                                                           unsigned long nSteps) {
    // construct intermediate waypoints in path between the two poses
    std::vector<geometry_msgs::Pose> steps;
    double t = 0;
    double Rfrom, Pfrom, Yfrom;
    double Rto, Pto, Yto;
    double Rdiff, Pdiff, Ydiff;
    poseToYPR(from, &Yfrom, &Pfrom, &Rfrom);
    poseToYPR(to, &Yto, &Pto, &Rto);

    // remove extra rotations of the gripper
    Rdiff = (Rto - Rfrom);
    while (Rdiff > PI)
        Rdiff -= PI;
    while (Rdiff < -PI)
        Rdiff += PI;

    Pdiff = (Pto - Pfrom);
    while (Pdiff > PI)
        Pdiff -= PI;
    while (Pdiff < -PI)
        Pdiff += PI;

    Ydiff = (Yto - Yfrom);
    while (Ydiff > PI)
        Ydiff -= PI;
    while (Ydiff < -PI)
        Ydiff += PI;

    // step measures to add
    double step[] = {(Rto - Rfrom) / nSteps,
                     (Pto - Pfrom) / nSteps,
                     (Yto - Yfrom) / nSteps};

    for (int idx = 1; idx < nSteps; idx++) {
        t = double(idx) / nSteps;
        geometry_msgs::Pose intermedStep;
        intermedStep.position.x = ((1 - t) * from.position.x) + (t * to.position.x);
        intermedStep.position.y = ((1 - t) * from.position.y) + (t * to.position.y);
        intermedStep.position.z = ((1 - t) * from.position.z) + (t * to.position.z);
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(Rfrom + idx * step[0],
                                                          Pfrom + idx * step[1],
                                                          Yfrom + idx * step[2]),
                              intermedStep.orientation);
        steps.push_back(intermedStep);
    }
    steps.push_back(to);
    return steps;
}

// Gripper
void G01Gripper::gripperOpen() {
    if (!sim) {
        ROS_INFO_STREAM("5 SEC TO MAN OPEN THE GRIPPER");
        ros::Duration(5).sleep();
        //fakeGripperOpen();
        return;
    }

    // code to be used with robotiq gripper
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

void G01Gripper::fakeGripperOpen() {
    std_msgs::UInt16 command;
    command.data = 16;
    fakeGripperCommandPub.publish(command);
}

bool G01Gripper::gazeboDetach(std::string name, std::string link) {
    gazebo_ros_link_attacher::AttachRequest req;
    req.model_name_1 = "robot";
    req.link_name_1 = "wrist_3_link";
    req.model_name_2 = std::move(name);
    req.link_name_2 = std::move(link);
    gazebo_ros_link_attacher::AttachResponse res;
    return detacher.call(req, res);
}

void G01Gripper::gripperClose(int howMuch) {
    if (!sim) {
        ROS_INFO_STREAM("5 SEC TO MAN CLOSE THE GRIPPER");
        ros::Duration(5).sleep();
        //fakeGripperClose();
        return;
    }

    // code to be used with robotiq gripper
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

void G01Gripper::fakeGripperClose() {
    std_msgs::UInt16 command;
    command.data = 150;
    fakeGripperCommandPub.publish(command);
}

bool G01Gripper::gazeboAttach(std::string name, std::string link) {
    gazebo_ros_link_attacher::AttachRequest req;
    req.model_name_1 = "robot";
    req.link_name_1 = "wrist_3_link";
    req.model_name_2 = std::move(name);
    req.link_name_2 = std::move(link);
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
                collObjects.emplace_back(
                        addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id));
            } else if (vol[0] * 2 == vol[2]) {
                cylToGrab.emplace_back(item);
                item.pose.position.z -= vol[2] / 2;
                collObjects.emplace_back(
                        addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id));
            } else {
                // pose correction for triangle prisms
                // to center the point the gripper points to
                tf::Vector3 translation(0, -0.03, 0);
                tf::Transform P(tf::createIdentityQuaternion(), translation);

                tf::Transform initial, final;
                tf::poseMsgToTF(item.pose, initial);
                final = initial * P;
                tf::poseTFToMsg(final, item.pose);

                item.pose.position.z -= vol[2] / 4;

                triToGrab.emplace_back(item);
                collObjects.emplace_back(
                        addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id, true));
            }
            grabObjNames.emplace_back(item.header.frame_id);
        }
    }
    ROS_INFO_STREAM("Grab cubes " << cubeToGrab.size() << " cyls " << cylToGrab.size() << " tris " << triToGrab.size());
}

void G01Gripper::avoidCB(const g01_perception::PoseStampedArray::ConstPtr &input) {
    for (geometry_msgs::PoseStamped item: input->poses) {
        if (!(std::find(grabObjNames.begin(), grabObjNames.end(), item.header.frame_id) != grabObjNames.end())) {
            objectsToAvoid.emplace_back(item);
            grabObjNames.emplace_back(item.header.frame_id);
        }
        std::vector<float> vol = getVolume(item.header.frame_id); // space in x,y,z

        if (vol[0] == vol[2] || vol[0] * 2 == vol[2])
            collObjects.emplace_back(
                    addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id));
        else {
            // pose correction for triangle prisms
            tf::Vector3 translation(0, -0.03, 0);
            tf::Transform P(tf::createIdentityQuaternion(), translation);

            tf::Transform initial, final;
            tf::poseMsgToTF(item.pose, initial);
            final = initial * P;
            tf::poseTFToMsg(final, item.pose);

            item.pose.position.z -= vol[2] / 4;
            collObjects.emplace_back(
                    addCollisionBlock(item.pose, vol[0], vol[1], vol[2], item.header.frame_id, true));
        }
    }
    ROS_INFO_STREAM("Avoid objects " << objectsToAvoid.size());
}

moveit_msgs::CollisionObject G01Gripper::addCollisionBlock(geometry_msgs::Pose pose,
                                                           float Xlen, float Ylen, float Zlen, std::string objectId,
                                                           bool isTriangle) {
    // object initialization
    moveit_msgs::CollisionObject co;
    co.header.frame_id = planFrameId;
    co.id = std::move(objectId);
    co.operation = co.ADD;

    if (!isTriangle) {
        // add a box-like shape as collision object
        // base the shape on lengths around the pose
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
        shapes::ShapeMsg meshMsg;
        shapes::constructMsgFromShape(m, meshMsg);
        mesh = boost::get<shape_msgs::Mesh>(meshMsg);

        // resize the mesh and place on the pose
        co.meshes.resize(1);
        co.mesh_poses.resize(1);
        co.meshes[0] = mesh;
        co.mesh_poses[0].position = pose.position;
        co.mesh_poses[0].orientation = pose.orientation;

        double y, p, r;
        poseToYPR(pose, &y, &p, &r);
        r = 0, p = 0;

        // rotate the previous pose by 45* about Z because of mesh orientation
        tf::Quaternion qOriginal, qRotation, qFinal;
        qOriginal = tf::createQuaternionFromRPY(r, p, y);
        r = 0, p = 0, y = PI / 2;
        qRotation = tf::createQuaternionFromRPY(r, p, y);
        qRotation.normalize();

        // calculate the new orientation
        qFinal = qOriginal * qRotation;
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
    collObjects.emplace_back(addCollisionBlock(backWall, 0.1, 1.3, 2, "back_wall"));

    geometry_msgs::Pose sideWall;
    sideWall.position.x = 0.15;
    sideWall.position.y = -0.8;
    sideWall.position.z = 1;
    collObjects.emplace_back(addCollisionBlock(sideWall, 2, 0.1, 2, "side_wall"));
}

// Utilities
void G01Gripper::poseToYPR(geometry_msgs::Pose pose, double *yaw, double *pitch, double *roll) {
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(pose.orientation, quaternion);
    tf::Matrix3x3 mat(quaternion);
    mat.getEulerYPR(*yaw, *pitch, *roll);
}

bool G01Gripper::isHeld(int howMuch) {
    // check if object is held by gripper's fingers
    if (sim) return true;

    return true; // fixme test on real one (come concordato con Elisa)
    /*
    ros::Duration(1).sleep();

    // all three in the same position but equal to howMuch
    if ((int) status.gPOA >= howMuch &&
        (int) status.gPOB == (int) status.gPOC &&
        (int) status.gPOB == (int) status.gPOA)
        return false;

    // all fingers in a position != from the previous
    int precA = -1, precB = -1, precC = -1;
    while ((int) status.gPOA != precA && (int) status.gPOB != precB && (int) status.gPOC != precC) {
        precA = status.gPOA;
        precB = status.gPOB;
        precC = status.gPOC;
        ros::Duration(0.2).sleep();
    }
    return true;

    //alternative method not working on simulation

    while(status.gSTA == 0)
    ros::Duration(0.2).sleep();
    return status.gSTA == 1 || status.gSTA == 2;
    return = false -> (status.gSTA == 3) -> close more
    */
}

void G01Gripper::gripperCB(const robotiq_s_model_control::SModel_robot_input &msg) {
    // just save status of the gripper to be globally used
    status = msg;
}

void G01Gripper::goHome(moveit::planning_interface::MoveGroupInterface &group) {
    // just to be sure
    gripperOpen();

    // set joint values and move
    group.setJointValueTarget(HOME_JOINT_POS);
    if (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        group.move();
}

void G01Gripper::marrOdomCallback(const nav_msgs::Odometry::ConstPtr &OdomPose) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf2_listener(tfBuffer);
    try {
        geometry_msgs::TransformStamped odom_to_world;
        odom_to_world = tfBuffer.lookupTransform("world", "marrtino_map", ros::Time(0), ros::Duration(10.0));
        tf2::doTransform(OdomPose->pose.pose, LZPose, odom_to_world);
        odomReceived = true;
        marrOdomSub.shutdown();
    } catch (tf2::TransformException &exception) {
        ROS_ERROR_STREAM("Odom-World transform not found");
    }
}