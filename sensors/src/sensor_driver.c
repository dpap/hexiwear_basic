/*
 * sensor_driver.c
 *
 *
 *      Author: dp
 */
#include "sensor_driver.h"

static void sensor_GetData( task_param_t param );
static void sensor_InitModules();

static mutex_t
  taskAccessMutex;

task_handler_t
  hexiwear_sensor_TAG_handler;
/**
 * create the sensor task
 */
osa_status_t sensor_Init()
{
	osa_status_t
		osaStatus = kStatus_OSA_Success;

	// create a mutex for controlling the access to toggling the task
	osaStatus = OSA_MutexCreate( &taskAccessMutex );

	if ( kStatus_OSA_Success != osaStatus )
	{
		catch( CATCH_MUTEX );
		return SENSOR_STATUS_ERROR;
	}

	// create a semaphore for signalizing when to acquire new sensor readings
	//osaStatus = sensor_CreateBinSema( &sensor_sema );

	if ( kStatus_OSA_Success != osaStatus )
	{
		catch( CATCH_SEMAPHORE );
		return SENSOR_STATUS_ERROR;
	}
	sensor_InitModules();

	osaStatus = OSA_TaskCreate  (
									sensor_GetData,
									(uint8_t*)"Sensor Task Get Data",
									0x1000,
									NULL,
									HEXIWEAR_SENSOR_PRIO,
									(task_param_t)0,
									false,
									&hexiwear_sensor_TAG_handler
								);

	if ( kStatus_OSA_Success != osaStatus )
	{
		catch( CATCH_INIT );
		return SENSOR_STATUS_ERROR;
	}

	return osaStatus;
}

static void sensor_InitModules()
{
	fxos_status_t fxosStatus = FXOS_Init( &fxosModule, &fxosSettings );
	if ( STATUS_FXOS_SUCCESS != fxosStatus )
	{
	  catch( CATCH_INIT );
	}
}

/**
 * retrieve data from various sensors
 * @param param task parameter, currently unused
 */
static void sensor_GetData( task_param_t param )
{
	fxos_status_t fxosStatus;
	motionData_t motionData;
	 while (1)
	    {
		 uint8_t sensorStatus;

		 	fxosStatus = FXOS_ReadStatus( &sensorStatus );

		 if ( 0x00 == sensorStatus )
		 {
			fxosStatus = STATUS_FXOS_ERROR;
		 }
		 else
		 {
		   // check also if tap is enabled
		   if ( false == FXOS_CheckForTap() )
		   {
			 fxosStatus = STATUS_FXOS_ERROR;
		   }
		   else
		   {
			  fxosStatus = FXOS_ReadRawData( (int16_t*)&motionData );
		   }
		 }
		 OSA_TimeDelay( 500 );
	    }

}
