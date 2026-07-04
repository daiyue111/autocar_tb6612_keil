#ifndef TRACK_H
#define TRACK_H

#include "ti_msp_dl_config.h"

uint8_t track_read_mask(void);
uint8_t track_black_count(uint8_t mask);
bool track_has_line(void);
int16_t track_error(uint8_t mask);

#endif
