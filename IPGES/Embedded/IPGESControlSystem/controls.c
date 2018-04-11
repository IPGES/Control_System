//*****************************************************************************
//
// led_task.c - A simple flashing LED task.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "spi_task.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "controls.h"
#include "adc_task.h"

/*
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
*/

//*****************************************************************************
//
// The stack size for the task.
//
//*****************************************************************************
#define SPITASKSTACKSIZE        128         // Stack size in words

//*****************************************************************************
//
// The item size and queue size for the message queue.
//
//*****************************************************************************
#define SPI_ITEM_SIZE           sizeof(uint8_t)
#define SPI_QUEUE_SIZE          5

//*****************************************************************************
//
// The queue that holds messages sent to the LED task.
//
//*****************************************************************************
//xQueueHandle g_pSpiQueue;

//*****************************************************************************
//
// Number of bytes to send and receive.
//
//*****************************************************************************
#define NUM_SSI_DATA            1


extern xSemaphoreHandle g_pUARTSemaphore;
//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************

static void ControlsTask(void *pvParameters)
{

		xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
		UARTprintf("Control Init\n");
		xSemaphoreGive(g_pUARTSemaphore);
    portTickType ui32WakeTime;
	
    // Get the current tick count.
    ui32WakeTime = xTaskGetTickCount();
	
    // Loop forever.
    while(1)
    {  
			/*
			xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
			UARTprintf("Freq %d\n", input);
			xSemaphoreGive(g_pUARTSemaphore); */
			
			//***********************************Capacitor bank controls*************************************************
			// Double check variable types
			
			int measured_load_voltage = get_dist_v_rms(); // un-hardcode, ask Jim for the voltage measurement, or just change the if statements below.
			int target_voltage = 24 ; // distribution line Vin voltage setpoint
			int tolerance = 1; // distribution line Vin voltage tolerance
			int low_range = target_voltage-tolerance; 
			int high_range = target_voltage+tolerance; 
			
			if(measured_load_voltage<low_range){
				GPIO_CAP1_set_high();
				if(measured_load_voltage<low_range){
					GPIO_CAP2_set_high();
					if(measured_load_voltage<low_range){
						GPIO_CAP3_set_high();
					}
				}
			}
			if(measured_load_voltage>high_range){
				GPIO_CAP1_set_low();
				if(measured_load_voltage>high_range){
				GPIO_CAP2_set_low();
					if(measured_load_voltage>high_range){
					GPIO_CAP3_set_low();
					}
				}
			}
			//**************************************End capacitor controls****************************************//
			
			//**************************************Power - Load controls*****************************************//
			// Controlling load power levels during grid-tied scenario for demand response
			// Future work: can be altered by replacing voltage with power once the solar and wind Vin, Iin 
			// 							measurements are available.
			
			int controls(void){
				int target_voltage = 24; // what do we want this to be at? Less than 90% load
				// int measured_load_current = get_load_i_rms; // ask jim which of the current measurements this is
				int measured_load_voltage = get_dist_v_rms();
				int derivative;
				int error;
				int integral;
				int last_error = 0; // unsure how this being 0 will affect the system
				int delta_time; // get delta time from Jim
				int pwm;
				int dt; //
				int kp = 1; // proportional gain
				int ki = 1; // integral gain
				int kd = 1; // derivative gain
				int control_variable= 0;
			
			/*
			 // Here we should decide if we want to modify generation or load. In the first case, we will only be changing load.
			 // Calculate the error, or delta between the target frequency and the newly measured frequency.
			 error = target_voltage - measured_power; //P
			 integral += error*dt;//I
			 derivative = (error - last_error)/delta_time; // D -- unsure about delta in time
			 // Calculate the control variable
			 control_variable = kp*error + ki*integral + kd*derivative;
			 
			 // include something for open loop protections
		 
		//   // Limit the duty cycle to never be 0 to 100; important for
		//   if(control_variable >= target_frequency){
		//    pwm = control_variable/30;
		//   }
		//   if(control_variable <= target_frequency){}
		//   if(pwm < 5) {pwm = 5;}
		//  else if(pwm < 95) {pwm = 95;} */
		  //**************************************End of load controls*****************************************//
			
			
			
			ADC_PrintJSON(); //needed for UI
			//ADC_Print();
			vTaskDelayUntil(&ui32WakeTime, 1000 / portTICK_RATE_MS); //Sleep Scheduler
			
    } //forever loop 
}

//*****************************************************************************
//
// Initializes the PWM task to output a PWM to PB6 and it's complement to PB7.
//
//*****************************************************************************
uint32_t ControlsTaskInit(void)
{
    // Create the task.
    if(xTaskCreate(ControlsTask, (const portCHAR *)"Controls", SPITASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SPI_TASK, NULL) != pdTRUE) 
    {
        return(1);
    }
    // Success.
    return(0);
}

