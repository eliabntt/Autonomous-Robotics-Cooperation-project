//
// Created by rig8f on 30/01/19.
//

#ifndef G01_FSM_STATES_H
#define G01_FSM_STATES_H

const unsigned short
        STATE_MARR_RUN  = 0, // marrtino moving in the arena
        STATE_UR10_WAKE = 1, // wake up perception to scan for objects
        STATE_UR10_RDY  = 2, // ur10 ready to move objects
        STATE_UR10_LOAD = 3, // objects load start command
        STATE_UR10_TBC  = 4, // load ok but not finished: another round is needed
        STATE_UR10_DONE = 5; // load ok: finish

const std::string
        STATE_TOPIC = "/g01_fsm_state", // state transmitter
        START_TOPIC = "/g01_start_run"; // to start when another run is needed

const double STATE_SLEEP_TIME = 0.3; // wait time when fsm checks/loops are involved

#endif //G01_FSM_STATES_H
