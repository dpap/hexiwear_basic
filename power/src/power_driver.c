/*
 * power_driver.c
 *
 *  Created on: 13 May 2017
 *      Author: dp
 */


#include "power_driver.h"
#include "PWR_Manager.h"
#include "sensor_driver.h"

#define BLUE_LED_ON()   GPIO_DRV_ClearPinOutput( BLUE_LED );
#define BLUE_LED_OFF()   GPIO_DRV_SetPinOutput( BLUE_LED );
/**
 * call before entering sleep mode
 * @param notify optional parameters
 * @param callbackData optional parameters
 * @return status flag
 */
power_manager_error_code_t power_CallBeforeSleep(
													power_manager_notify_struct_t *notify,
													power_manager_callback_data_t* callbackData
												)
{
 //   currentWakeSrc = POWER_WAKE_SRC_NONE;

    /** enable UART edge interrupt circuit, */
    UART_WR_BDH_RXEDGIE( g_uartBase[ 4 ] , 1 );

    /** post to semaphore to make it available to interrupts */
 //   OSA_SemaPost( &power_wakeSrcSema );

    BLUE_LED_OFF();
	return kPowerManagerSuccess;
}

/**
 * call after exiting sleep mode
 * @param notify optional parameters
 * @param callbackData optional parameters
 * @return flag
 */
power_manager_error_code_t power_CallAfterSleep (
													power_manager_notify_struct_t *notify,
													power_manager_callback_data_t* callbackData
												)
{
    /** disable UART edge interrupt circuit */
    UART_WR_BDH_RXEDGIE( g_uartBase[4] , 0 );

    BLUE_LED_ON();
    //read bat level after power up
    forceGetBatteryLevel();
    //fix issue with Accel not working after power down
    sensor_InitModules();
    return kPowerManagerSuccess;
}

/**
 * put MCU to VLPS mode,
 * enable/disable possible interrupts
 * @return status flag
 */
power_status_t power_PutMCUToSleep()
{
    bool
        sensorTimerState = false;

    /** check for active communication */

    /** I2C */

    i2c_status_t
      currentStatus;

    uint32_t
      rxBytesRemaining = 0,
      txBytesRemaining = 0;

    for ( uint8_t i2cIdx = 0; i2cIdx < 2; i2cIdx++ )
    {
        currentStatus = kStatus_I2C_Success;
        currentStatus |= I2C_DRV_MasterGetSendStatus   ( i2cIdx, &txBytesRemaining );
        currentStatus |= I2C_DRV_MasterGetReceiveStatus( i2cIdx, &rxBytesRemaining );
        if  (
                    ( kStatus_I2C_Success == currentStatus )
                &&  ( 0 != ( txBytesRemaining + rxBytesRemaining ) )
            )
        {
            return POWER_STATUS_INIT_ERROR;
        }
    }

	POWER_SYS_SetMode( 1, kPowerManagerPolicyAgreement );
    //POWER_SYS_SetMode( 1, kPowerManagerPolicyForcible);
	CLOCK_SYS_UpdateConfiguration( 0, kClockManagerPolicyForcible );

	OSA_TimeDelay(10);

}
