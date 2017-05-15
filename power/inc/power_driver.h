/*
 * power_driver.h
 *
 *  Created on: 13 May 2017
 *      Author: dp
 */

#ifndef POWER_INC_POWER_DRIVER_H_
#define POWER_INC_POWER_DRIVER_H_

#include <stdbool.h>
#include "fsl_power_manager.h"
#include "power_types.h"
#include "fsl_i2c_master_driver.h"
/**
 * call before entering sleep mode
 * @param notify optional parameters
 * @param callbackData optional parameters
 * @return status flag
 */
power_manager_error_code_t power_CallBeforeSleep(
													power_manager_notify_struct_t* notify,
													power_manager_callback_data_t* callbackData
												);

/**
 * call after exiting sleep mode
 * @param notify optional parameters
 * @param callbackData optional parameters
 * @return flag
 */
power_manager_error_code_t power_CallAfterSleep (
													power_manager_notify_struct_t* notify,
													power_manager_callback_data_t* callbackData
												);

power_status_t power_PutMCUToSleep();
#endif /* POWER_INC_POWER_DRIVER_H_ */
