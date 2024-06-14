#ifndef SENSORZSSC3224_SENSOR_H_
#define SENSORZSSC3224_SENSOR_H_

#include "common_utils.h"
#if (BSP_CFG_RTOS > 0)
#include "sensor_thread.h"
#endif
#include "sensor_zssc3224/sensor_zssc3224.h"

void g_zssc3224_quick_setup(void);
void g_zssc3224_sensor0_read(float *sensor_data, float *temperature_data);

#endif /* ZSSC3224_SENSOR_H_ */
