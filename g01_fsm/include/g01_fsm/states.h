//
// Created by rig8f on 30/01/19.
//

#ifndef G01_FSM_STATES_H
#define G01_FSM_STATES_H

const int ERROR = -1,
        MARR_FWD = 0,  // marrtino going to the loading zone
        MARR_BWD = 1,  // marrtino going to the unloading point
        UR10_LOAD = 2, // piece load start command
        UR10_TBC  = 3, // load ok: another round is needed
        UR10_DONE = 4; // load ok: finish

#endif //G01_FSM_STATES_H
