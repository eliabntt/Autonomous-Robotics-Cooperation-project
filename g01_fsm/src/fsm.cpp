#include "boost/algorithm/string.hpp"
#include "ros/ros.h"
//#include "g01_perception/tags.h"

bool sim = true;
//ros::Publisher pubGrab, pubAvoid;

void initParam(ros::NodeHandle node_handle) {
    // evaluate node's given parameters
    if (node_handle.hasParam("sim")) {
        if (!node_handle.getParam("sim", sim)) {
            ROS_WARN_STREAM("Not valid 'sim' parameter's value. Setting 'sim' to 'true'.");
            sim = true;
        }

        ROS_INFO_STREAM((sim ? "Simulation..." : "Testing on real robot..."));
    } // else not necessary: it has a default value

    /*if (node_handle.hasParam("ids")) {
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
    }*/
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "g01_fsm");

    // parse parameters
    ros::NodeHandle n("~");
    initParam(n);

    // todo empty for now

    return 0;
}
