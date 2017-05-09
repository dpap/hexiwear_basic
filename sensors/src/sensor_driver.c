/*
 * sensor_driver.c
 *
 *
 *      Author: dp
 */
#include "sensor_driver.h"
#include "host_mcu_interface.h"

static bool
    isPowerActive_HTU_TSL = false,
	isPowerActive_BATTERY = false,
	isPowerActive_MAXIM   = false;

#define PWR_HTU_TSL_TurnON()  GPIO_DRV_ClearPinOutput( PWR_SENSORS_NF ); OSA_TimeDelay( 50 ); isPowerActive_HTU_TSL = true
#define PWR_HTU_TSL_TurnOFF() isPowerActive_HTU_TSL = false; GPIO_DRV_SetPinOutput( PWR_SENSORS_NF )

#define PWR_HR_TurnON()  GPIO_DRV_SetPinOutput( PWR_HR ); OSA_TimeDelay( 50 ); isPowerActive_MAXIM = true
#define PWR_HR_TurnOFF() isPowerActive_MAXIM = false; GPIO_DRV_ClearPinOutput( PWR_HR )

#define PWR_BATT_TurnON()  GPIO_DRV_ClearPinOutput( PWR_BAT_SENS ); OSA_TimeDelay( 50 ); isPowerActive_BATTERY = true
#define PWR_BATT_TurnOFF() isPowerActive_BATTERY = false; GPIO_DRV_SetPinOutput( PWR_BAT_SENS )

#define TypeMember_NumEl( type, member ) ( sizeof( ( (type*)0 )->member ) / sizeof( ( (type*)0 )->member[0] ) )

static void sensor_GetData( task_param_t param );
static void sensor_InitModules();
void sensors_power();

static mutex_t
  taskAccessMutex;

task_handler_t
  hexiwear_sensor_TAG_handler;
/**
 * create the sensor task
 */
osa_status_t sensor_Init()
{
	sensors_power();

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

			   //assemble packet for BLE
			   void *dataStart = NULL;
			   // data packet to be sent
			   hostInterface_packet_t sensorPacket;

			   sensorPacket.type = packetType_accel;

			   dataStart           = (void*)&( motionData.accData[0] );
			   sensorPacket.length = TypeMember_NumEl( motionData_t ,  accData );
			   sensorPacket.length *= sizeof( mE_t );

			   sensorPacket.data[0] =  motionData.accData[0] & 0xFF;
			   sensorPacket.data[1] = ( motionData.accData[0] >>8) & 0xFF;
			   sensorPacket.data[2] =  motionData.accData[1] & 0xFF;
			   sensorPacket.data[3] = ( motionData.accData[1] >>8) & 0xFF;
			   sensorPacket.data[4] =  motionData.accData[2] & 0xFF;
			   sensorPacket.data[5] = ( motionData.accData[2] >>8) & 0xFF;

			   // add trailer byte, denoting the packet end
			   sensorPacket.data[ sensorPacket.length ] = gHostInterface_trailerByte;

			   osa_status_t hostStatus =  HostInterface_TxQueueMsgPut( &sensorPacket, false );
			   if (hostStatus == kStatus_OSA_Success){
				  asm ("nop");
			   } else {
				   //error, queue probably full
				  asm ("nop");
			   }

		   }
		 }
		 OSA_TimeDelay( 10 );
	    }

}

void sensors_power(){
	PWR_HTU_TSL_TurnOFF();
	PWR_HR_TurnOFF();
	PWR_BATT_TurnOFF();
}
