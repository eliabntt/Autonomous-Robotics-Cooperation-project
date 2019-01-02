//
// Created by rig8f on 26/12/18.
//
#include <fstream>
#include "g01_move.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "g01_move");

    G01Move m = G01Move();

    ros::waitForShutdown();
    return 0;
}

