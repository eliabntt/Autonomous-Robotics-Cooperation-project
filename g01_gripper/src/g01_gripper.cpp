//
// Created by eliabntt on 28/11/18.
//

#include <g01_gripper.h>

#include "g01_gripper.h"


G01Gripper::G01Gripper() {
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //robot
    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    //scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());


    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = 0.001;
    target_pose1.orientation.y = 0.007107;
    target_pose1.position.x = 0.0037766;
    target_pose1.position.y = -0.0043118;
    target_pose1.position.z = -0.00549799;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);


    ROS_INFO_STREAM("tutorial" << "Visualizing plan 1 (pose goal) %s" << (success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "" :
                                                                    "FAILED"));
    move_group.move();
}

bool G01Gripper::move(double x, double y) {
    return false;
}




