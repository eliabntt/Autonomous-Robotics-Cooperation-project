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
#include "g01_perception/PoseStampedArray.h"
#include "../include/tags.h"

std::vector<std::string> params;

bool forever = false, sim = true, isIdentity = false;
ros::Publisher pubGrab, pubAvoid;
std::fstream outputFile;

bool fileInit() {
    // set output file name and open a stream on it
    std::string filePath = ros::package::getPath("g01_perception") + "/output.txt";
    outputFile = std::fstream(filePath, std::fstream::out);
    if (!(outputFile.is_open())) {
        ROS_ERROR_STREAM("ERROR: failure opening file, data won't be saved");
        return false;
    }
    return true;
}

void fileTagWrite(apriltags_ros::AprilTagDetection tag) {
    // write a tag to the file
    outputFile << "tag id: " << tag.id << std::endl
               << "frame id: " << tagnames[tag.id] << std::endl;
    outputFile << "    size: " << tag.size << std::endl;
    outputFile << "    orientation: x = " << tag.pose.pose.orientation.x
               << "  y = " << tag.pose.pose.orientation.y
               << "  z = " << tag.pose.pose.orientation.z
               << "  w = " << tag.pose.pose.orientation.w << std::endl;
    outputFile << "    position: x = " << tag.pose.pose.position.x
               << "  y = " << tag.pose.pose.position.y
               << "  z = " << tag.pose.pose.position.z << std::endl << std::endl;
}

geometry_msgs::TransformStamped transform(const std::string from, const std::string to) {
    // retrieve the transform between two frames
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    try {
        return tfBuffer.lookupTransform(to, from, ros::Time(0), ros::Duration(10)); //Time(0) = latest
    } catch (tf2::TransformException &exception) {
        ROS_WARN_STREAM(exception.what());

        // return an identity transform as a fallback
        isIdentity = true;
    	return tfBuffer.lookupTransform(from, from, ros::Time(0), ros::Duration(10));
    }
}

apriltags_ros::AprilTagDetection addOffset(apriltags_ros::AprilTagDetection tag) {
    // no offset in real measurements
    if (!sim)
        return tag;

    // offset to use when referring to camera_frame
    tag.pose.pose.position.x = tag.pose.pose.position.x - 0.01;
    tag.pose.pose.position.y = tag.pose.pose.position.y - 0.03;
    return tag;
}

void detectionsCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr &input) {
    // initialize stream to file
    bool isFileInit = false;
    if (!forever)
        isFileInit = fileInit();

    // initialize arrays to be published
    g01_perception::PoseStampedArray poseGrab, poseAvoid;
    poseGrab.header.stamp = ros::Time::now();
    poseAvoid.header.stamp = ros::Time::now();

    // initialize frame transform from camera to base
    geometry_msgs::TransformStamped camBaseTransform = transform("camera_rgb_optical_frame", "world");

    // identity matrix = some error occurred in the transform
    // set accordingly the rest of the header of the messages
    if (isIdentity) {
        poseGrab.header.frame_id = "/camera_rgb_optical_frame";
        poseAvoid.header.frame_id = "/camera_rgb_optical_frame";
    } else {
        poseGrab.header.frame_id = "/world";
        poseAvoid.header.frame_id = "/world";
    }

    // add a "header" to the file, before printing poses
    if (isFileInit)
        outputFile << "Detected frames, w.r.t. " << (isIdentity ? "camera" : "world") << " reference frame:\n";

    // loop through detections
    for (apriltags_ros::AprilTagDetection tag : input->detections) {

        // add offset (sim case is handled into the method)
        tag = addOffset(tag);

        //transform pose w.r.t world (only if lookup was successful)
        if (!isIdentity)
		tf2::doTransform(tag.pose.pose, tag.pose.pose, camBaseTransform);

        int idt = tag.id;
        geometry_msgs::PoseStamped ps = tag.pose;
        ps.header.frame_id = tagnames[idt]; // overwrite to contain tag name
        if (std::find(params.begin(), params.end(), tagnames[idt]) != params.end()) {
            // (requested) tag found over objects on the table: write on file
            ROS_INFO_STREAM("tag id: " << idt << " = " << tagnames[idt]);
            if (isFileInit)
                fileTagWrite(tag);

            // add poseStamped to vectors
            poseGrab.poses.emplace_back(ps);
        } else
            poseAvoid.poses.emplace_back(ps);
    }

    // close stream to file
    if (isFileInit)
        outputFile.close();

    // publish PoseArrays
    pubAvoid.publish(poseAvoid);
    pubGrab.publish(poseGrab);

    // exit if in single-shot mode
    if (!forever)
        ros::shutdown();
}

void initParam(ros::NodeHandle node_handle) {
    // evaluate node's given parameters
    if (node_handle.hasParam("sim")) {
        if (!node_handle.getParam("sim", sim)) {
            ROS_WARN_STREAM("Not valid 'sim' parameter's value. Setting 'sim' to 'true'.");
            sim = true;
        }

        ROS_INFO_STREAM((sim ? "Simulation..." : "Testing on real robot..."));
    } // else not necessary: it has a default value

    if (node_handle.hasParam("forever")) {
        if (!node_handle.getParam("forever", forever)) {
            ROS_WARN_STREAM("Not valid 'forever' parameter's value. Setting 'forever' to 'false'.");
            forever = false;
        }

        ROS_INFO_STREAM("Scan will " << (forever ? "not end."
                                                 : "run only for one detection. \n File \"output.txt\" will be created."));
    }

    if (node_handle.hasParam("ids")) {
        // extract list of ids (objects to grab)
        std::string tmp;
        std::vector<std::string> ids;

        // get string, remove spaces and split by ','
        node_handle.getParam("ids", tmp);
        boost::erase_all(tmp, " ");
        boost::split(ids, tmp, boost::is_any_of(","));

        std::string empty("");
        for (auto i:ids) {
            // exclude empty strings
            if (i == empty)
                continue;

            if (std::find(tagnames.begin(), tagnames.end(), i) != tagnames.end())
                params.emplace_back(i);
            else
                ROS_WARN_STREAM(i << " is NOT a valid tag or keyword");
        }

        // if params is still empty, flag all tags as valid
        if (params.empty()) {
            ROS_INFO_STREAM("No (valid) ids passed, all flagged as valid.");
            params = tagnames;
        }
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "g01_perception");

    // parse parameters
    ros::NodeHandle n("~");
    initParam(n);

    ROS_INFO_STREAM("Detection starting...\n\n");
    // subscribe to receive detections
    ros::Subscriber sub = n.subscribe<apriltags_ros::AprilTagDetectionArray>("/tag_detections", 100,
                                                                             detectionsCallback);
    // initialize publishers of results
    pubGrab = n.advertise<g01_perception::PoseStampedArray>("/tags_to_grab", 1000);
    pubAvoid = n.advertise<g01_perception::PoseStampedArray>("/tags_to_avoid", 1000);

    // in single-shot mode, a single spinOnce call won't work, so
    // repeat it until first detection message arrives, then stop;
    // in forever mode, rate will be maintained
    ros::Rate rate(100); // expressed in Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
