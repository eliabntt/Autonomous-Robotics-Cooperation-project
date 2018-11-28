//
// Created by eliabntt on 28/11/18.
//

#include <g01_gripper.h>

#include "g01_gripper.h"

std::string planFrameId, endEffId;

G01Gripper::G01Gripper() : command(), n(){
    //fixme maybe 4 better?
    ros::AsyncSpinner spinner(2);
    spinner.start();

    //init

    //todo implement better for wrong inputs(see hw1)
    sim = n.hasParam("sim") ? n.getParam("sim", sim) : true;


    //robot
    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface my_group(PLANNING_GROUP);
    //gripper
    gripperCommandPub = n.advertise<robotiq_s_model_control::SModel_robot_output>("/robotiq_hands/l_hand/SModelRobotOutput",1);

    //todo mmmm maybe see this
    const robot_state::JointModelGroup* joint_model_group =
        my_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //some basic info
    planFrameId = my_group.getPlanningFrame();
    endEffId = my_group.getEndEffectorLink();
    ROS_INFO_STREAM("Reference frame: " << planFrameId);
    ROS_INFO_STREAM("End effector frame: " << endEffId);
    geometry_msgs::PoseStamped c = my_group.getCurrentPose(endEffId);
    ROS_INFO_STREAM(c);

    //todo targe home position to fix the joint links
    geometry_msgs::Pose target_pose1;
    tf::Quaternion q_rot;
    double r=0, p=3.145/2, y=0;
    q_rot = tf::createQuaternionFromRPY(r, p, y);
    tf::quaternionTFToMsg(q_rot,target_pose1.orientation);

    target_pose1.position.x = 0.37;
    target_pose1.position.y = -0.12;
    target_pose1.position.z = 1.6;
    my_group.setPoseTarget(target_pose1);

    //todo complete - limit but seems not to work so use the collision https://answers.ros.org/question/273485/move_groupsetworkspace-doesnt-work/
    // my_group.setWorkspace(-1,-1,1,1,1,3);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (my_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    my_group.move();
    ROS_INFO_STREAM("Visualizing plan 1 (pose goal)" << (success ? "" : "FAILED"));


    close(255);
    //todo cycle this for initialization of the workspace and also for the single small objects
    //OVERWRITE ONLY THE POSITION

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    geometry_msgs::Pose myobj;
    myobj.position.x = 1;
    myobj.position.y = 22;
    myobj.position.z = 1;
    myobj.orientation.x = 0;
    myobj.orientation.y = 0;
    myobj.orientation.z = 0;
    myobj.orientation.w = 1;
    collision_objects.emplace_back(collision(myobj, 0.4,0.6,0.2, "ciao"));

    //todo out of the cycle
    planning_scene_interface.addCollisionObjects(collision_objects);

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

void G01Gripper::open() {
    command.rACT = 1;
    command.rMOD= 0;
    command.rGTO= 1;
    command.rATR= 0;
    command.rGLV= 0;
    command.rICF= 0;
    command.rICS= 0;
    command.rPRA= 0;
    command.rSPA= 0;
    command.rFRA= 0;
    command.rPRB= 0;
    command.rSPB= 0;
    command.rFRB= 255;
    command.rPRC= 0;
    command.rSPC= 0;
    command.rFRC= 0;
    command.rPRS= 0;
    command.rSPS= 0;
    command.rFRS= 0;
    gripperCommandPub.publish(command);
}

//todo probably will need different howMuch based on the object
void G01Gripper::close(int howMuch) {
    assert(howMuch > 0);
    command.rACT = 1;
    command.rMOD= 0;
    command.rGTO= 1;
    command.rATR= 0;
    command.rGLV= 0;
    command.rICF= 0;
    command.rICS= 0;
    command.rPRA= (unsigned char) howMuch;
    command.rSPA= 255;
    command.rFRA= 150;
    command.rPRB= 0;
    command.rSPB= 0;
    command.rFRB= 0;
    command.rPRC= 0;
    command.rSPC= 0;
    command.rFRC= 0;
    command.rPRS= 0;
    command.rSPS= 0;
    command.rFRS= 0;
    gripperCommandPub.publish(command);
}


moveit_msgs::CollisionObject G01Gripper::collision(geometry_msgs::Pose pose, double length, double width, double height, std::string obj_id)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = planFrameId;
    collision_object.id = obj_id;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = length;
    primitive.dimensions[1] = width;
    primitive.dimensions[2] = height;
    geometry_msgs::Pose box_pose;
    box_pose = pose;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    return collision_object;
}