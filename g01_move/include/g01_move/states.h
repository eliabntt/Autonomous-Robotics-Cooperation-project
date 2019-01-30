//
// Created by rig8f on 30/01/19.
//

#ifndef G01_FSM_STATES_H
#define G01_FSM_STATES_H

const unsigned short
        STATE_MARR_RUN  = 0, // marrtino moving in the arena
        STATE_UR10_WAKE = 1, // wake up perception to scan for objects
        STATE_UR10_LOAD = 2, // objects load start command
        STATE_UR10_TBC  = 3, // load ok but not finished: another round is needed
        STATE_UR10_DONE = 4; // load ok: finish

const std::string STATE_TOPIC = "/g01_fsm_state";

#endif //G01_FSM_STATES_H
