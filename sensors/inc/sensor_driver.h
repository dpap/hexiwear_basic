/*
 * sensor_driver.h
 *
 *
 *      Author: dp
 */

#ifndef SENSORS_INC_SENSOR_DRIVER_H_
#define SENSORS_INC_SENSOR_DRIVER_H_

#include "fsl_os_abstraction.h"

#include "HEXIWEAR.h"

#include "sensor_types.h"
#include "error.h"

#include "FXOS_driver.h"
/**
 * create the sensor task
 */
osa_status_t sensor_Init();
void sensor_InitModules();
uint16_t showDistance();

void flash_SensorInit();
void flash_SensorDeInit();

uint16_t showBatteryLevel();
void forceGetBatteryLevel();

void pauseSensorTask();
void resumeSensorTask();
#endif /* SENSORS_INC_SENSOR_DRIVER_H_ */
