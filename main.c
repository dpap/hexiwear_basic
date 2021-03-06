/**
 * @file main.c
 * @brief this file contains the launch of the initialization task which will
 * finish the necessary initialization upon the scheduler is started
 *
 * @version 1.00
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * Neither the name of NXP, nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * visit: http://www.mikroe.com and http://www.nxp.com
 *
 * get support at: http://www.mikroe.com/forum and https://community.nxp.com
 *
 * Project HEXIWEAR, 2015
 */

#include "main.h"
#include "OLED_defs.h"
#include "OLED_driver.h"
#include "OLED_resources.h"
#include "agile.h"
#include "sensor_driver.h"
#include "power_driver.h"
#include "rtc_driver.h"

static bool isPowerActive_OLED    = false;
static task_handler_t powerOled_taskHandler;
static bool guiControlLeft = false;

#define PWR_OLED_TurnON()  GPIO_DRV_SetPinOutput( PWR_OLED ); OSA_TimeDelay( 50 ); isPowerActive_OLED = true
#define PWR_OLED_TurnOFF() isPowerActive_OLED = false; GPIO_DRV_ClearPinOutput( PWR_OLED )

void drawAgileScreen();
void resetOLED_dynamicArea();

void Task1( task_param_t param )
{
    oled_dynamic_area_t oled_dynamic_area;
    oled_text_properties_t oled_text_properties;

    hostInterface_packet_t command_packet;

	OLED_Init( &oledModule, &oledSettings );
	OLED_FillScreen( GUI_COLOR_BLACK );

	oled_dynamic_area.xCrd = 0;
	oled_dynamic_area.yCrd = 0;
	oled_dynamic_area.width = 96;
	oled_dynamic_area.height = 96;
	OLED_SetDynamicArea( &oled_dynamic_area );

	oled_text_properties.font = guiFont_Tahoma_8_Regular;
	oled_text_properties.fontColor = GUI_COLOR_WHITE;
	oled_text_properties.alignParam = OLED_TEXT_ALIGN_LEFT;
	oled_text_properties.background = NULL;
	OLED_SetTextProperties( &oled_text_properties );

	OLED_DrawText( "Welcome" );

	drawAgileScreen();

	while(1)
	{
		OSA_TimeDelay( 500 );

		HostInterface_CmdQueueMsgGet( &command_packet );

		switch( command_packet.type )
		{

		    // buttons
		    case packetType_pressUp:
		    {
		    	if (guiControlLeft == true) {
		    		OLED_DrawText( "Toggle Bluetooth" );
		    		bluetooth_SendToggleAdvModeReq();
		    		guiControlLeft = false;
		    	} else {
		    		OLED_DrawText( "Offline on" );
		    		flash_SensorInit();
		    	}
				vTaskResume( powerOled_taskHandler);
				break;
		    }
		    case packetType_pressDown:
		    {

				OLED_DrawText( "Offline off" );
				flash_SensorDeInit();

				vTaskResume( powerOled_taskHandler);
				break;
		    }
		    case packetType_pressLeft:
		    {
		    	guiControlLeft = true;
		    	//OLED_DrawText( "Left" );
		    	//drawAgileScreen();
		    	char buffer [20];
		    	oled_dynamic_area.yCrd = 0;
		    	oled_dynamic_area.height = 48;
		    	OLED_SetDynamicArea( &oled_dynamic_area );
		    	rtc_datetime_t watch_time;
		    	RTC_GetCurrentTime(&watch_time);
		    	snprintf( (char*)buffer, 12, "Time: %02d:%02d", watch_time.hour, watch_time.minute);
		    	OLED_DrawText(&buffer);

		    	oled_dynamic_area.yCrd = 48;
		    	oled_dynamic_area.height = 47;
		    	OLED_SetDynamicArea( &oled_dynamic_area );
		    	forceGetBatteryLevel();
		    	uint16_t batLevel = (uint16_t) ( (showBatteryLevel()  * (4200))  /56000);
		    	sprintf(buffer, "Batt: %lu mV",batLevel);
		    	OLED_DrawText(&buffer);

		    	oled_dynamic_area.yCrd = 0;
		    	oled_dynamic_area.height = 96;
		    	OLED_SetDynamicArea( &oled_dynamic_area );

				vTaskResume( powerOled_taskHandler);
		    	break;
		    }
		    case packetType_pressRight:
		    {
		    	OLED_DrawText( "Right" );
				vTaskResume( powerOled_taskHandler);
				char buffer [20];
				sprintf(buffer, "Activity %lu ", showDistance());
				OLED_DrawText(&buffer);

				break;
		    }
		    case packetType_passDisplay:
		    {
		    	vTaskResume( powerOled_taskHandler);
		    	char buffer [20];
		    	uint32_t passkey = 0;
		    	memcpy(&passkey, command_packet.data, 3);

		    	sprintf(buffer, "Pin %lu", passkey);
		    	OLED_DrawText(&buffer);
		    }
		     case packetType_advModeSend:
		      {
		    	  enum
				  {
					bluetooth_advMode_disable  = 0,
					bluetooth_advMode_enable   = 1,
				  } bluetooth_advMode;
				  bluetooth_advMode = command_packet.data[0];
				  if (bluetooth_advMode == bluetooth_advMode_disable) {
					  OLED_DrawText("Bluetooth Disabled");
				  } else if (bluetooth_advMode == bluetooth_advMode_enable){
					  OLED_DrawText("Bluetooth Enabled");
				  }
		      }
		    default: {}
	    }
	}
}


void powerOLED( task_param_t param )
{
	while (1){
		OSA_TimeDelay( 10000 );
		PWR_OLED_TurnOFF();
		power_PutMCUToSleep();
		vTaskSuspend( powerOled_taskHandler);
		PWR_OLED_TurnON();
	}
}

void drawAgileScreen(){
	oled_status_t
		  statusOLED = OLED_STATUS_SUCCESS;
	uint8_t
		  xCrd = 0,
		  yCrd = 0,
		  width,
		  height;

	      OLED_GetImageDimensions( &width, &height, (const uint8_t*)agile96_bmp );

  while (1) {
	statusOLED = OLED_DrawScreen(
     								agile96_bmp,
                                     xCrd,
                                     yCrd,
                                     width,
                                     height,
										OLED_TRANSITION_NONE // transition
                                 );
     if ( OLED_STATUS_SUCCESS == statusOLED )
     {
         break;
     }
  }
}


void main()
{
  /** initialize the hardware */
  PE_low_level_init();

  /** disable write buffering end enable ARM exceptions */
  HEXIWEAR_EnableExceptions();

  /** initialize the startup task */
  HEXIWEAR_Init();

  OSA_TaskCreate(
		          Task1,
                  (uint8_t*)"Task1",
                  0x1000,
                  NULL,
                  1,
                  (task_param_t)0,
                  false,
                  NULL
                );

  OSA_TaskCreate(
		          powerOLED,
                  (uint8_t*)"PowerOLED",
                  0x1000,
                  NULL,
                  0,
                  (task_param_t)0,
                  false,
				  &powerOled_taskHandler
                );

  /** start RTOS scheduler */
  HEXIWEAR_Start();

  while (1) {}
}


