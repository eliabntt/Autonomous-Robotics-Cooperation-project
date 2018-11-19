#include <fstream>
#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "apriltags_ros/AprilTagDetectionArray.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Vector3.h"
#include "boost/algorithm/string.hpp"

std::vector<std::string> params;
const std::vector<std::string> tagnames = {
        "red_cube_1", "red_cube_2", "red_cube_3", "red_cube_4",
        "yellow_cyl_1", "yellow_cyl_2",
        "green_triangle_1", "green_triangle_2", "green_triangle_3",
        "blue_cube_1", "blue_cube_2", "blue_cube_3", "blue_cube_4",
        "red_triangle_1", "red_triangle_2", "red_triangle_3"
};

bool forever = false;
bool sim = true;
bool is_identity = false;
ros::Publisher pubGrab, pubAvoid;

geometry_msgs::TransformStamped transform(const std::string from, const std::string to) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    try {
        return tfBuffer.lookupTransform(to, from, ros::Time(0), ros::Duration(0.1)); //Time(0) latest
    } catch (tf2::TransformException &exception) {
        ROS_WARN_STREAM(exception.what());
        ros::Duration(0.5).sleep();
        is_identity = true;
    }
    return tfBuffer.lookupTransform(from, from, ros::Time(0), ros::Duration(0.1));
}

apriltags_ros::AprilTagDetection addOffset(apriltags_ros::AprilTagDetection tag) {
    if (!sim) return tag;
    if (!is_identity) {
        tag.pose.pose.position.x = tag.pose.pose.position.x - 0.01;
        tag.pose.pose.position.y = tag.pose.pose.position.y + (tag.size) / 2;
        tag.pose.pose.position.z = tag.pose.pose.position.z + 0.0075;
        return tag;
    } else {
        tag.pose.pose.position.x = tag.pose.pose.position.x - 0.01;
        tag.pose.pose.position.y = tag.pose.pose.position.y - 0.03;
        return tag;
    }
}

void detectionsCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr &input) {
    // set output file name and open a stream on it
    std::string file_path = ros::package::getPath("hw1_perception") + "/output.txt";
    std::fstream output_file(file_path, std::fstream::out);
    if (!(output_file.is_open()))
        ROS_ERROR_STREAM("ERROR: failure opening file, data won't be saved");


    // initialize array to be published
    geometry_msgs::PoseArray poseGrab, poseAvoid;
    poseGrab.header.stamp = ros::Time::now();
    poseAvoid.header.stamp = ros::Time::now();

    // initialize frame transform from camera to base
    geometry_msgs::TransformStamped camBaseTransform = transform("camera_rgb_optical_frame", "base_link");

    //if it's identity we have some problems in the transform
    if (is_identity) {
        poseGrab.header.frame_id = "/camera_rgb_optical_frame";
        poseAvoid.header.frame_id = "/camera_rgb_optical_frame";
        output_file << "Detected frames, w.r.t. camera reference frame:\n";
    } else {
        poseGrab.header.frame_id = "/base_link";
        poseAvoid.header.frame_id = "/base_link";
        output_file << "Detected frames, w.r.t. base reference frame:\n";
    }

    // loop through detections
    for (apriltags_ros::AprilTagDetection tag : input->detections) {

        // transform pose w.r.t base_link
        if (!is_identity)
            tf2::doTransform(tag.pose.pose, tag.pose.pose, camBaseTransform);

        tag = addOffset(tag);

        int idt = tag.id;
        if (std::find(params.begin(), params.end(), tagnames[idt]) != params.end()) {
            // tag found over objects on the table
            ROS_INFO_STREAM("tag id: " << idt << " = " << tagnames[idt]);

            output_file << "tag id: " << idt << std::endl
                        << "frame id: " << tagnames[idt] << std::endl;
            output_file << "    size: " << tag.size << std::endl;
            output_file << "    orientation: x = " << tag.pose.pose.orientation.x
                        << "  y = " << tag.pose.pose.orientation.y
                        << "  z = " << tag.pose.pose.orientation.z
                        << "  w = " << tag.pose.pose.orientation.w << std::endl;
            output_file << "    position: x = " << tag.pose.pose.position.x
                        << "  y = " << tag.pose.pose.position.y
                        << "  z = " << tag.pose.pose.position.z << std::endl << std::endl;

            // add pose to vectors
            poseGrab.poses.emplace_back(tag.pose.pose);
        } else {
            poseAvoid.poses.emplace_back(tag.pose.pose);
        }
    }
    output_file.close();

    // publish PoseArrays
    pubAvoid.publish(poseAvoid);
    pubGrab.publish(poseGrab);

    if (!forever) // exit if in single-shot mode
        ros::shutdown();
}


void initParam(ros::NodeHandle node_handle) {
    if (node_handle.hasParam("sim")) {
        node_handle.getParam("sim", sim);
        if (sim)
            ROS_INFO_STREAM("Simulation...");
        else
            ROS_INFO_STREAM("Testing on real robot...");
    } else {
        ROS_ERROR("Failed to get param 'sim' (boolean). Setting 'sim' to 'true'.");
        sim = true;
    }

    if (node_handle.hasParam("forever")) {
        ROS_INFO_STREAM("Forever parameter detected. Scan will not end.");
        forever = true;
    } else forever = false;

    if (node_handle.hasParam("ids")) {

        std::string tmp;
        std::vector<std::string> ids;
        node_handle.getParam("ids", tmp);
        boost::erase_all(tmp, " ");
        boost::split(ids, tmp, boost::is_any_of(","));

        if (tmp.length() > 0)
            for (auto i:ids) {
                if (std::find(tagnames.begin(), tagnames.end(), i) != tagnames.end())
                    params.emplace_back(i);
                else
                    ROS_INFO_STREAM(i << " is NOT a valid tag or keyword");
            }
        else {
            ROS_INFO_STREAM("No ids passed, all flagged as valid..");
            params = tagnames;
        }
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "hw1_perception");


    // subscribe to receive detections
    ros::NodeHandle n("~");

    initParam(n);

    ros::Subscriber sub = n.subscribe<apriltags_ros::AprilTagDetectionArray>("/tag_detections", 100,
                                                                             detectionsCallback);
    // initialize publishers of results
    pubGrab = n.advertise<geometry_msgs::PoseArray>("/tags_to_grab", 1000);
    pubAvoid = n.advertise<geometry_msgs::PoseArray>("/tags_to_avoid", 1000);

    // in single-shot mode, a single spinOnce call won't work,
    // so repeat it until first detection message arrives, then stop;
    // in forever mode, rate will be maintained
    ros::Rate rate(5); // expressed in Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

