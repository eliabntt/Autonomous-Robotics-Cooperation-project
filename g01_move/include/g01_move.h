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

    // useful resources http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/
    // http://www.theconstructsim.com/ros-qa-126-how-to-configure-the-differential-drive-ros-controller/
    // https://answers.ros.org/question/289561/help-to-run-diff_drive_controller/
    // https://books.google.it/books?id=skFPDwAAQBAJ&pg=PA182&lpg=PA182&dq=two+wheeled+robot+ros+yaml+config&source=bl&ots=fgpCIFfQSa&sig=90Tbzlv0YMDtm_aLxqukdQby_Q4&hl=en&sa=X&ved=2ahUKEwiYh4CXocrfAhXF-aQKHTjqBvIQ6AEwBXoECAUQAQ#v=onepage&q=two%20wheeled%20robot%20ros%20yaml%20config&f=false

    // http://wiki.ros.org/navigation/Tutorials/RobotSetup#Costmap_Configuration_.28local_costmap.29_.26_.28global_costmap.29
    // https://answers.ros.org/question/279148/costmap-parameters-navigation-stack/
    // https://answers.ros.org/question/219108/ghost-obstacles-on-local-costmap/
    // https://answers.ros.org/question/277769/how-to-decrease-obstacle-area-in-costmaps/

    // this is TOO CLOSE so the opposite of what we have now https://answers.ros.org/question/268485/navigation-planning-too-near-to-obstacles/
    // https://answers.ros.org/question/12874/costmap2d-inflation-radius/

    //fixme
    /*
    STATIC VALUES
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