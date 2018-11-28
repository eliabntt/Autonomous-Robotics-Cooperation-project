//
// Created by eliabntt on 28/11/18.
//

#include <g01_gripper.h>

#include "g01_gripper.h"

G01Gripper::G01Gripper() : command(), n(){
    ros::AsyncSpinner spinner(2);
    spinner.start();

    //init
    //todo implement better for wrong inputs(see hw1)
    sim = n.hasParam("sim") ? n.getParam("sim", sim) : true;
    gripperCommandPub = n.advertise<robotiq_s_model_control::SModel_robot_output>("/robotiq_hands/l_hand/SModelRobotOutput",1);


    //robot
    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface my_group(PLANNING_GROUP);

    //scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //todo mmmm
    const robot_state::JointModelGroup* joint_model_group =
        my_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    //basic info
    ROS_INFO("Reference frame: %s", my_group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", my_group.getEndEffectorLink().c_str());


    //todo targe home position to fix the joint links

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = -0.05;
    target_pose1.orientation.y = 0.8;
    target_pose1.orientation.z = 0.03;
    target_pose1.orientation.w = 0.4;
    target_pose1.position.x = 0.37;
    target_pose1.position.y = -0.12;
    target_pose1.position.z = 1.6;
    my_group.setPoseTarget(target_pose1);

    //todo complete - limit
    //my_group.setWorkspace()

    geometry_msgs::PoseStamped c = my_group.getCurrentPose("ee_link");
    ROS_INFO_STREAM(c);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (my_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    my_group.move();

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


    close(255);

    spinner.stop();

/*
    if(sim)
        my_group.detachObject(id);

    if(sim)
        my_group.attatchObject(id, my_group.getEndEffectorLink().c_str());
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

//todo probably will need different howMuch
void G01Gripper::close(int howMuch) {
    assert(howMuch > 0);
    //if is active
    ROS_INFO_STREAM("a");
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
    ROS_INFO_STREAM("b");

}