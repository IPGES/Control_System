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
    //char uartInput[20]; 

	
		uint16_t input = 0;
	
    // Loop forever.
    while(1)
    {  
		input = ADC_PrintFreq();	
		
			xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("Freq %d\n", input);
		xSemaphoreGive(g_pUARTSemaphore);
        //
        // Wait for the required amount of time.
        //
        vTaskDelayUntil(&ui32WakeTime, 1000 / portTICK_RATE_MS); 
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

