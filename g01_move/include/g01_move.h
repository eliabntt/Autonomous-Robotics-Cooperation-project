//
// Created by eliabntt on 28/11/18.
//
#include <utility>
#include <ros/ros.h>

#ifndef G01_MOVE_G01_MOVE_H
#define G01_MOVE_G01_MOVE_H

class G01Move {
public:
    G01Move();

private:
    // MEMBERS
    bool sim;
    ros::NodeHandle n;

    //fixme
    /*
    loading_pose
    unloading_pose
    start_corridor_pose
    end_corridor_pose
    */

    /*
    REPEAT
    1 -> start corridor
     a. -> start wall following https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_WALLFOLLOWING
                                https://github.com/ssscassio/ros-wall-follower-2-wheeled-robot
            since I
            have only to go straight and to check the distance from the front wall
    2 -> loading
     a. -> get distance from end wall and side wall
     b. -> publish for LZ branch
    3. -> end corridor(rotate and position the robot)
    4. -> start corrdior
    5. -> unloading pose
    */

};

#endif