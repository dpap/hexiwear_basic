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


#endif /* SENSORS_INC_SENSOR_DRIVER_H_ */
