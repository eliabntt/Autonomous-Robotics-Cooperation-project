#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/package.h"

#include <fstream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

std::vector<std::string> params;
const std::vector<std::string> tagnames = {
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
bool stop = false;

void detectionsPoseCallback(const geometry_msgs::PoseArray::ConstPtr &input) {
    // todo: save to file instead of this
    if (!input->poses.empty()) {
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


void detectionsCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr &input) {
    std::string file_path = ros::package::getPath("hw1_perception")+"/output.txt";
    std::fstream output_file(file_path, std::fstream::out);
    if (!(output_file.is_open())){
        ROS_INFO_STREAM("error: failed opening file, data won't be saved");
    }
    output_file << "detected frames:\n";
    for (apriltags_ros::AprilTagDetection tag : input->detections) {
        std::vector<double> temp = std::vector<double>();
        int idt = int(tag.id);
        if (std::find(params.begin(), params.end(), tagnames[idt]) != params.end()) {
            ROS_INFO_STREAM(idt);
            output_file << "tag " << idt << " name: " << tagnames[idt] << "\n";
            output_file << "    size: " << tag.size << "\n";
            output_file << "    orientation: x=" << tag.pose.pose.orientation.x
                << " y= " << tag.pose.pose.orientation.y
                << " z= " << tag.pose.pose.orientation.z
                << " w= " << tag.pose.pose.orientation.w << "\n";
            output_file << "    position: x=" << tag.pose.pose.position.x
                << " y= " << tag.pose.pose.position.y
                << " z= " << tag.pose.pose.position.z << "\n";
        }
    }
    output_file.close();
    stop = true;
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "discover");


    if (argc == 1)
        params = tagnames;
    else {
        for (int i = 1; i < argc; i++) {
            if (std::find(tagnames.begin(), tagnames.end(), argv[i]) != tagnames.end())
                params.emplace_back(argv[i]);
            else
                ROS_INFO_STREAM(argv[i] << " is NOT a valid tag");
        }
    }

    ros::NodeHandle n;
    ros::Subscriber sub_comprehensive = n.subscribe<apriltags_ros::AprilTagDetectionArray>("/tag_detections", 1,
                                                                                           detectionsCallback);

    while (ros::ok() && !stop) {
        ros::spinOnce();
    }
    return 0;
}
