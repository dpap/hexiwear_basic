/*
 * sensor_driver.c
 *
 *
 *      Author: dp
 */
#include "math.h"
#include "sensor_driver.h"
#include "host_mcu_interface.h"

#include "FLASH_driver.h"
#include "PEx.h"

static mutex_t
  taskAccessMutex;

task_handler_t
  hexiwear_sensor_TAG_handler;

static msg_queue_handler_t
  sensorProc_txQueueHnd;

static mutex_t
  sensorProcessMutex;

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

#define SAMPLE_PERIOD 1000
#define WINDOWSIZE 10; //

int16_t accData[3];
int16_t accDataPrev[3] = {0,0,0};
static int16_t distance=0;
static uint32_t accDistAverage = 0;
static uint16_t accCount=0;

static uint16_t window = WINDOWSIZE;
static uint16_t distanceAverage=0;

#define BAT_SAMPLE_TICK 30 // Period = BAT_SAMPLE_TICK * SAMPLE_PERIOD
static uint8_t batSamplingCounter = 0;
static uint16_t batLevel = 0;

/** message queues */
MSG_QUEUE_DECLARE( sensorProc_txQueue, 5, sizeof(accData) );

static void sensor_GetData( task_param_t param );
static void sensor_InitModules();
void sensors_power();
static void sensor_InitProcessing();
static void sensor_ProcessData( task_param_t param );
uint16_t calculateParameters(uint16_t accData[3],uint16_t accDataPrev[3]);

void flash_writeSensorData(uint8_t* data, uint32_t length);
uint32_t flashFindNextValidSector();
uint32_t flashBlockInit(uint32_t address);

uint16_t getBatteryLevel();
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
	sensor_InitProcessing();

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

	osaStatus = OSA_TaskCreate  (
									sensor_ProcessData,
									(uint8_t*)"Sensor Task Process Data",
									0x1000,
									NULL,
									0,
									(task_param_t)0,
									false,
									NULL
								);

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
	 batLevel =  getBatteryLevel();

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

				BaseType_t
				      status = xQueueSendToBack(sensorProc_txQueueHnd, &motionData.accData, OSA_WAIT_FOREVER );
				    if ( pdPASS == status )	    {
				    	 asm ("nop");
				    } else {
				    	 asm ("nop");
				    }

		   }
		 }
			 OSA_TimeDelay( SAMPLE_PERIOD );
			 if (batSamplingCounter >= BAT_SAMPLE_TICK){
				 batLevel =  getBatteryLevel();
				 batSamplingCounter=0;
			 } else{
				 batSamplingCounter ++;
			 }
	     }

}

void sensors_power(){
	PWR_HTU_TSL_TurnOFF();
	PWR_HR_TurnOFF();
	//PWR_BATT_TurnOFF();
}

//Process sensor data
static void sensor_InitProcessing(){

	osa_status_t status = OSA_MutexCreate(&sensorProcessMutex);

		if( kStatus_OSA_Success != status )
		{
			catch( CATCH_QUEUE ) ;
		}
	  // create TX message queue
      int16_t accData[3];
	  sensorProc_txQueueHnd = OSA_MsgQCreate (
			  	  	  	  	  	  	  	  	  	  sensorProc_txQueue,
	                                              5,
	                                              sizeof(accData)
	                                            );
	  if ( NULL == sensorProc_txQueueHnd )
	  {
		  catch(2);
	  }

}
/**
 * Process data from sensors
 */
static void sensor_ProcessData( task_param_t param ){

	int16_t accData[3];
	while (1) {
	    osa_status_t
	        status = OSA_MsgQGet(sensorProc_txQueueHnd, &accData, OSA_WAIT_FOREVER);
	      if ( kStatus_OSA_Error == status ){
	        catch(2);
	      } else {
	        OSA_MutexLock( &sensorProcessMutex, OSA_WAIT_FOREVER );

	    			distance = calculateParameters(accData,accDataPrev);
	    			accDataPrev[0] = accData[0]; accDataPrev[1] = accData[1]; accDataPrev[2] = accData[2];

					if (accCount >= window){
						distanceAverage =accDistAverage/window;
						accCount =0;
						accDistAverage=0;
						int16_t distAverageArr[1];
						distAverageArr[0]=distanceAverage;
						flash_writeSensorData(&distAverageArr,sizeof(accDistAverage));
					} else {
						accCount++;
						accDistAverage += distance;
					}
	    	OSA_MutexUnlock(&sensorProcessMutex);
	      }
	}
}

uint16_t calculateParameters(uint16_t accData[3],uint16_t accDataPrev[3]){

	//Distance
	 int16_t x,y,z,x2,y2,z2;

	 x=(int16_t) accData[0];
	 y=(int16_t) accData[1];
	 z=(int16_t) accData[2];
	 x2=(int16_t) accDataPrev[0];
	 y2=(int16_t) accDataPrev[1];
	 z2=(int16_t) accDataPrev[2];

	float distance = sqrt( pow((x2-x),2) + pow((y2-y),2) + pow((z2-z),2));

	distance *= 100;
	//uint16_t test = sqrt(25);
	asm("nop");
	return distance;
}

uint16_t showDistance(){
	return distanceAverage;
}


//Flash storage
/**
 *
 */
#define  SECTOR0  		0x00000000L
#define  SECTOR1  		0x00001000L
#define  SECTOR2  		0x00002000L
#define  SECTOR2047     0x007FF000L
#define  END_OF_FLASH   0x007FFFFFL
#define  SECTORSIZE     4096
#define  MAX_SAMPLES    1350000L
#define  bufferStartAddressPointer 0x00000010L

const uint32_t bufferEndAddress 	= SECTOR2 + SECTORSIZE -1;

static uint32_t flash_active = 0;
static uint32_t bufferStartAddress	= SECTOR2;
static uint32_t flash_currentAddress = 0x00002000L;

void flash_SensorInit(){
	uint32_t bufferCurrentAddress ;
	//read magic numbers at start of flash
	uint8_t data[3] = {0,0,0};
	FLASH_ReadData(0, &data,3);
	if (data[0] != 0x33 && data[1] != 0x44 && data[2] != 0x55) {
		FLASH_EraseSector(SECTOR0);
		data[0]=0x33; data[1] = 0x44; data[2] = 0x55;
		FLASH_WriteData(0,  &data,3);
		data[0]=0x0; data[1] = 0x0; data[2] = 0x0;
		FLASH_ReadData(0, &data,3);
		if (data[0] == 0x33 && data[1] == 0x44 && data[2] == 0x55){
			bufferStartAddress = SECTOR2;
			FLASH_WriteData(bufferStartAddressPointer, &bufferStartAddress,sizeof(bufferStartAddress));
			flash_currentAddress = bufferStartAddress;
			flash_currentAddress = flashBlockInit(flash_currentAddress);
			flash_active = 1;
		} else {
			flash_active = 0;
			return;
		}
	} else {
		flash_currentAddress = flashFindNextValidSector();
		if ((flash_currentAddress != 0xFFFFFFFF) && (flash_currentAddress < END_OF_FLASH) ) {
			bufferStartAddress = flash_currentAddress;
			flash_currentAddress = flashBlockInit(flash_currentAddress);
			flash_active = 1;
		} else {
			flash_active = 0;
			return;
		}
		asm("nop");
	}

}

void flash_SensorDeInit(){
	flash_active = 0;
}

uint32_t flashBlockInit(uint32_t address){
	uint32_t timestamp;
	uint32_t nextAddress = address;

	window = WINDOWSIZE;

	FLASH_ReadData(address,&timestamp,sizeof(timestamp) );
	if (timestamp != 0xFFFFFFFF) {
		FLASH_EraseSector(address);
	}
	/* TODO implement RTC
	rtc_datetime_t watch_time;
	RTC_GetCurrentTime(&watch_time);
	RTC_HAL_ConvertDatetimeToSecs( &watch_time, &timestamp );
	*/
	timestamp = 1494332579;
	FLASH_WriteData(address,&timestamp,sizeof(timestamp));

	nextAddress = address + sizeof(address) + 2;
	asm("nop");

	return nextAddress;
}

uint32_t flashFindNextValidSector(){
	uint32_t timestamp = 0xFFFFFFFF;
	for (uint32_t addr = SECTOR2; addr <= SECTOR2047 ; addr = addr + 4096 ){
		FLASH_ReadData(addr, &timestamp, sizeof(timestamp));
		if (timestamp == 0xFFFFFFFF) return addr;
	}
	return 0xFFFFFFFF;
}

void flash_writeSensorData(uint8_t data[], uint32_t length){
	if (flash_active == 0 ){
			return;
	} else if (flash_active > MAX_SAMPLES){
		flash_active = 0;
	}
	else {
		flash_active++;
	}

	if (flash_currentAddress == 0) {
		flash_active = 0;
		asm("nop");
		return;
	}

	if ( (flash_currentAddress < bufferStartAddress) || (flash_currentAddress > END_OF_FLASH -1 )){
		flash_active = 0;
	} else if ( flash_currentAddress > bufferStartAddress + SECTORSIZE - 1 - length){
		bufferStartAddress = (flash_currentAddress/4096 + 1) * 4096;
	    flash_currentAddress = flashBlockInit(bufferStartAddress);
	    FLASH_WriteData(flash_currentAddress,data,length);
	    flash_currentAddress += length;
	} else {
		FLASH_WriteData(flash_currentAddress,data,length);
		flash_currentAddress += length;
	}
}

uint16_t getBatteryLevel(){

	uint16_t adcData ;
    ADC16_DRV_ConfigConvChn( FSL_BATTERY_ADC, 0, &BATTERY_ADC_ChnConfig);
    ADC16_DRV_WaitConvDone ( FSL_BATTERY_ADC, 0 );
    adcData = ADC16_DRV_GetConvValueSigned( FSL_BATTERY_ADC, 0 );
    ADC16_DRV_PauseConv(FSL_BATTERY_ADC, 0 );
	return adcData;
}

uint16_t showBatteryLevel(){
	return batLevel;
}

void forceGetBatteryLevel(){
	batLevel =  getBatteryLevel();
}
