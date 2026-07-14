#ifndef TASK3_H
#define TASK3_H

#include "ti_msp_dl_config.h"

void task3_run(void);
bool task3_run_one_lap(void);
bool task3_run_one_lap_with_turns(int32_t aToAcTurnRaw,
    int32_t bToBdTurnRaw);

#endif
