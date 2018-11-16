#include <fstream>

#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "apriltags_ros/AprilTagDetectionArray.h"
#include <unistd.h>

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
bool forever = false;

void detectionsCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr &input) {
    // set output file name and open a stream on it
    std::string file_path = ros::package::getPath("hw1_perception") + "/output.txt";
    std::fstream output_file(file_path, std::fstream::out);
    if (!(output_file.is_open()))
        ROS_ERROR_STREAM("ERROR: failure opening file, data won't be saved");

    output_file << "Detected frames, w.r.t. Kinect reference frame:\n";
    for (apriltags_ros::AprilTagDetection tag : input->detections) {
        int idt = (int) tag.id;
        if (std::find(params.begin(), params.end(), tagnames[idt]) != params.end()) {
            // tag found over objects on the table
            ROS_INFO_STREAM(idt);

            output_file << "tag id: " << idt << std::endl
                        << "frame id: " << tagnames[idt] << std::endl;
            output_file << "    size: " << tag.size << std::endl; // todo FR is this value important?
            output_file << "    orientation: x = " << tag.pose.pose.orientation.x
                        << "  y = " << tag.pose.pose.orientation.y
                        << "  z = " << tag.pose.pose.orientation.z
                        << "  w = " << tag.pose.pose.orientation.w << std::endl;
            output_file << "    position: x = " << tag.pose.pose.position.x
                        << "  y = " << tag.pose.pose.position.y
                        << "  z = " << tag.pose.pose.position.z << std::endl << std::endl;
        }
    }
    output_file.close();
    stop = true; // useless in forever mode
}

int main(int argc, char *argv[]) {

    if (argc == 1) {
        ROS_INFO_STREAM("No parameters passed. Detecting all possible tags");
        params = tagnames;
    } else {
        for (int i = 1; i < argc; i++)
            // for each parameter, check if it is a frame id and save it in a list
            if (std::find(tagnames.begin(), tagnames.end(), argv[i]) != tagnames.end())
                params.push_back(argv[i]);
            else if (argv[i] == std::string("forever")) {
                ROS_INFO_STREAM("Forever parameter detected. Scan will not end.");
                forever = true;
            } else
                ROS_INFO_STREAM(argv[i] << " is NOT a valid tag or keyword");
    }

    ros::init(argc, argv, "discover");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<apriltags_ros::AprilTagDetectionArray>("/tag_detections", 100,
                                                                             detectionsCallback);

    if (forever)
        while (ros::ok()) {
            ros::spinOnce();
            usleep(250 * 1000); //four sample per second
        }
    else {
        // a single spinOnce call won't work, so repeating the call
        // until first detection message arrives, then stop
        while (ros::ok() && !stop) {
            ros::spinOnce();
            usleep(25 * 1000);
        }
    }

    return 0;
}
