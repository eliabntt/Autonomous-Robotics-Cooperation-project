//
// Created by eliabntt on 28/11/18.
//

#include <g01_gripper.h>

#include "g01_gripper.h"

G01Gripper::G01Gripper() : command(), n(){

    //init
    //todo implement better for wrong inputs(see hw1)
    sim = n.hasParam("sim") ? n.getParam("sim", sim) : true;

    ros::AsyncSpinner spinner(2);
    spinner.start();

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

    gripperCommandPub = n.advertise<robotiq_s_model_control::SModel_robot_output>("/robotiq_hands/l_hand/SModelRobotOutput",1);
    close(255);



    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = -0.25;
    target_pose1.position.y = 0.04;
    target_pose1.position.z = 1.5;
    my_group.setPoseTarget(target_pose1);

    //todo complete
    //my_group.setWorkspace()

    //todo use it for home function setting
    //my_group.getCurrentPose("ee_link");
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (my_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    my_group.move();

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


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

void G01Gripper::close(int howMuch) {
    assert(howMuch > 0);
    //if is active
    //todo complete
    if(command.rACT == 1)
    {
        ROS_INFO_STREAM("a");
        return;
    }
    command.rACT = 1;
    command.rMOD= 0;
    command.rGTO= 1;
    command.rATR= 0;
    command.rGLV= 0;
    command.rICF= 0;
    command.rICS= 0;
    command.rPRA= 255;
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
    ROS_INFO_STREAM("prova");
    return;
}