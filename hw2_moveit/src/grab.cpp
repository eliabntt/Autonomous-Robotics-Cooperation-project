#include <fstream>
#include "boost/algorithm/string.hpp"
#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Vector3.h"
#include "apriltags_ros/AprilTagDetectionArray.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

std::vector<std::string> params;
const std::vector<std::string> tagnames = {
        "red_cube_1", "red_cube_2", "red_cube_3", "red_cube_4",
        "yellow_cyl_1", "yellow_cyl_2",
        "green_triangle_1", "green_triangle_2", "green_triangle_3",
        "blue_cube_1", "blue_cube_2", "blue_cube_3", "blue_cube_4",
        "red_triangle_1", "red_triangle_2", "red_triangle_3"
};

bool forever = false, sim = true, isIdentity = false;


geometry_msgs::TransformStamped transform(const std::string from, const std::string to) {
    // retrieve the transform between two frames
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    try {
        return tfBuffer.lookupTransform(to, from, ros::Time(0), ros::Duration(0.1)); //Time(0) = latest
    } catch (tf2::TransformException &exception) {
        ROS_WARN_STREAM(exception.what());

        // return an identity transform as a fallback
        isIdentity = true;
        return tfBuffer.lookupTransform(from, from, ros::Time(0), ros::Duration(0.1));
    }
}

void detectionsCallback(const ) {


    // exit if in single-shot mode
    if (!forever)
        ros::shutdown();
}


void initParam(ros::NodeHandle node_handle) {

}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "hw2_moveit");

    // parse parameters
    ros::NodeHandle n("~");
    initParam(n);

    ROS_INFO_STREAM("Grabbing starting...\n\n");

    // subscribe to receive detections
    ros::Subscriber sub = n.subscribe<>();


    // in single-shot mode, a single spinOnce call won't work, so
    // repeat it until first detection message arrives, then stop;
    // in forever mode, rate will be maintained
    ros::Rate rate(5); // expressed in Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
