#ifndef IMU_H
#define IMU_H

#include "ti_msp_dl_config.h"

bool imu_init_gyro_z(void);
bool imu_read_gyro_z_delta(int32_t *delta);

#endif
