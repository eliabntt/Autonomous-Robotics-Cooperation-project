#include <fstream>
#include "g01_gripper.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "g01_gripper");

    G01Gripper r = G01Gripper();

    ros::waitForShutdown();
    return 0;
}
