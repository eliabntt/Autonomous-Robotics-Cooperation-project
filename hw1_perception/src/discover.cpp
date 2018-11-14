#include "ros/ros.h"
#include "std_msgs/String.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

std::vector<std::string> params;
std::vector<std::string> tagnames = {
        "red_cube_1",
        "red_cube_2",
        "red_cube_3",
        "red_cube_4",
        "yellow_cyl_1",
        "yellow_cyl_2",
        "green_triangle_1",
        "green_triangle_2",
        "green_triangle_3",
        "blue_cube_1",
        "blue_cube_2",
        "blue_cube_3",
        "blue_cube_4",
        "red_triangle_1",
        "red_triangle_2",
        "red_triangle_3"
};

void detectionsPoseCallback(const geometry_msgs::PoseArray::ConstPtr& input){
    // todo: save to file instead of this
    if (!input->poses.empty())
    {
      geometry_msgs::Pose output;
      output.position.x = input->poses[0].position.x;
      output.position.y = input->poses[0].position.y;
      output.position.z = input->poses[0].position.z;
      output.orientation.x = input->poses[0].orientation.x;
      output.orientation.y = input->poses[0].orientation.y;
      output.orientation.z = input->poses[0].orientation.z;
      output.orientation.w = input->poses[0].orientation.w;
    }
}


void detectionsCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& input){
    // todo: save to file instead of this
    for(apriltags_ros::AprilTagDetection tag : input->detections) {
        std::vector<double> temp = std::vector<double>();
        double idt = tag.id;
        if(std::find(params.begin(), params.end(), tagnames[int(idt)]) != params.end()) {
            temp.push_back(idt);
            temp.push_back(tag.size);
            temp.push_back(tag.pose.pose.orientation.x);
            temp.push_back(tag.pose.pose.orientation.y);
            temp.push_back(tag.pose.pose.orientation.z);
            temp.push_back(tag.pose.pose.orientation.w);
            temp.push_back(tag.pose.pose.position.x);
            temp.push_back(tag.pose.pose.position.y);
            temp.push_back(tag.pose.pose.position.z);
        }
    }
}

int main(int argc, char *argv[]){

    std::vector<std::string> args_vector(argv, argv + argc);
    params = args_vector;
    ros::init(argc, argv, "discover");
    ros::NodeHandle n;
    ros::Subscriber sub_comprehensive = n.subscribe<apriltags_ros::AprilTagDetectionArray>("/tag_detections", 1, detectionsCallback);

    ros::Subscriber sub_only_pose = n.subscribe<geometry_msgs::PoseArray>("/tag_detections_pose", 1, detectionsPoseCallback);

    ros::spin();
    return 0;
}
