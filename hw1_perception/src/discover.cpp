#include <fstream>
#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "apriltags_ros/AprilTagDetectionArray.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

std::vector<std::string> params;
const std::vector<std::string> tagnames = {
        "red_cube_1", "red_cube_2", "red_cube_3", "red_cube_4",
        "yellow_cyl_1", "yellow_cyl_2",
        "green_triangle_1", "green_triangle_2", "green_triangle_3",
        "blue_cube_1", "blue_cube_2", "blue_cube_3", "blue_cube_4",
        "red_triangle_1", "red_triangle_2", "red_triangle_3"
};

bool forever = false;
ros::Publisher tograb, toavoid, tograb_mine;

geometry_msgs::TransformStamped transform(const std::string from, const std::string to) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    try {
        return tfBuffer.lookupTransform(to, from, ros::Time(0), ros::Duration(10)); //Time(0) latest
    } catch (tf2::TransformException &exception) {
        ROS_WARN("%s", exception.what());
        ros::Duration(1.0).sleep();
    }
    return tfBuffer.lookupTransform(from, from, ros::Time(0), ros::Duration(10));
}

apriltags_ros::AprilTagDetection addOffset(apriltags_ros::AprilTagDetection tag) {
    // get transform between object and camera
    geometry_msgs::TransformStamped tagTransform = transform(tagnames[tag.id], "camera_rgb_optical_frame");


    // offset vector
    geometry_msgs::Vector3Stamped offset;
    offset.vector.x = (tag.size) / 2; // todo: check if the values make sense (orientation-wise)
    offset.vector.y = (tag.size) / 2;
    offset.vector.z = 0;

    // apply offset
    geometry_msgs::Vector3Stamped newPosition;
    tf2::doTransform(offset, newPosition, tagTransform);
    tag.pose.pose.position.x = newPosition.vector.x;
    tag.pose.pose.position.y = newPosition.vector.y;
    tag.pose.pose.position.z = newPosition.vector.z;
    return tag;
}

void detectionsCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr &input) {
    // set output file name and open a stream on it
    std::string file_path = ros::package::getPath("hw1_perception") + "/output.txt";
    std::fstream output_file(file_path, std::fstream::out);
    if (!(output_file.is_open()))
        ROS_ERROR_STREAM("ERROR: failure opening file, data won't be saved");

    //todo eventually change this wrt what we decide to publish
    output_file << "Detected frames, w.r.t. robot's Base reference frame:\n";

    // frame transform from camera to base
    geometry_msgs::TransformStamped camBaseTransform = transform("camera_rgb_optical_frame", "base_link");

    //initialize array to be published
    geometry_msgs::PoseArray poseGrab, poseAvoid, poseMine;
    poseGrab.header.stamp = ros::Time::now();
    poseGrab.header.frame_id = "/base_link";
    poseAvoid.header.stamp = ros::Time::now();
    poseAvoid.header.frame_id = "/base_link";
    poseMine.header.stamp = ros::Time::now();
    poseMine.header.frame_id = "/base_link";

    // loop through detections
    for (apriltags_ros::AprilTagDetection tag : input->detections) {
        // adding offset and referring all poses w.r.t. base link

        //  tag = addOffset(tag);

        int idt = tag.id;
        if (std::find(params.begin(), params.end(), tagnames[idt]) != params.end()) {
            // tag found over objects on the table
            ROS_INFO_STREAM("tag id: " << idt << " = " << tagnames[idt]);

            //todo decide what to output
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

            tf2::doTransform(tag.pose.pose, tag.pose.pose, camBaseTransform);

            poseGrab.poses.emplace_back(tag.pose.pose);
            //todo tune THIS
            tag.pose.pose.position.x = tag.pose.pose.position.x + (tag.size) / 2;
            tag.pose.pose.position.y = tag.pose.pose.position.y + (tag.size) / 2;
            tag.pose.pose.position.z = tag.pose.pose.position.z + (tag.size) / 2;
            poseMine.poses.emplace_back(tag.pose.pose);
        } else {
            //  tag = addOffset(tag);
            tf2::doTransform(tag.pose.pose, tag.pose.pose, camBaseTransform);

            //todo tune THIS and put it in a separate function
            tag.pose.pose.position.x = tag.pose.pose.position.x + (tag.size) / 2;
            tag.pose.pose.position.y = tag.pose.pose.position.y + (tag.size) / 2;
            tag.pose.pose.position.z = tag.pose.pose.position.z + (tag.size) / 2;

            poseAvoid.poses.emplace_back(tag.pose.pose);
        }
    }
    output_file.close();

    //publish PoseArray
    toavoid.publish(poseAvoid);
    tograb.publish(poseGrab);
    tograb_mine.publish(poseMine);


    if (!forever) // exit if in single-shot mode
        ros::shutdown();
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "discover");

    if (argc == 1) {
        ROS_INFO_STREAM("No parameters passed. Detecting all possible tags");
        params = tagnames;
    } else {
        for (int i = 1; i < argc; i++)
            // for each parameter, check if it is a frame id and save it in a list
            if (std::find(tagnames.begin(), tagnames.end(), argv[i]) != tagnames.end())
                params.emplace_back(argv[i]);
            else if (argv[i] == std::string("forever")) {
                ROS_INFO_STREAM("Forever parameter detected. Scan will not end.");
                forever = true;

                // handle when only forever is passed
                if (argc == 2)
                    params = tagnames;
            } else
                ROS_INFO_STREAM(argv[i] << " is NOT a valid tag or keyword");
    }

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<apriltags_ros::AprilTagDetectionArray>("/tag_detections", 100,
                                                                             detectionsCallback);
    tograb = n.advertise<geometry_msgs::PoseArray>("tags_to_grab", 1000);
    tograb_mine = n.advertise<geometry_msgs::PoseArray>("tags_to_grab_mine", 1000);
    toavoid = n.advertise<geometry_msgs::PoseArray>("tags_to_avoid", 1000);


    // in single-shot mode, a single spinOnce call won't work,
    // so repeat it until first detection message arrives, then stop
    // in forever mode, rate will be maintained
    ros::Rate rate(40); // expressed in Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
