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

#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"


#define GPIO_PORTB_DR8R_R       (*((volatile uint32_t *)0x40005508))
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
xQueueHandle g_pSpiQueue;

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

static void SPITask(void *pvParameters)
{
		uint32_t pui32DataTx[NUM_SSI_DATA];
    uint32_t pui32DataRx[NUM_SSI_DATA];
    uint32_t ui32Index;
	
		xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
		UARTprintf("SPI Init\n");
		xSemaphoreGive(g_pUARTSemaphore);
    portTickType ui32WakeTime;

		//
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
		while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx[0]))
    {
    }
	
    // Get the current tick count.
    ui32WakeTime = xTaskGetTickCount();
    //char uartInput[20]; 
	
	  pui32DataTx[0] = 128;
    //pui32DataTx[1] = '0';
    //pui32DataTx[2] = 0;
		
    // Loop forever.
    while(1)
    {  
			
			for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
			{
					//
					// Display the data that SSI is transferring.
					//
					//UARTprintf("'%c' ", pui32DataTx[ui32Index]);

					//
					// Send the data using the "blocking" put function.  This function
					// will wait until there is room in the send FIFO before returning.
					// This allows you to assure that all the data you send makes it into
					// the send FIFO.
					//
					SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
			}
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
uint32_t SPITaskInit(void)
{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		
		GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
		
		GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
								 GPIO_PIN_2);
	
		SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
									 SSI_MODE_MASTER, 976000, 8); // 1000000
	
		SSIEnable(SSI0_BASE);
	
    // Create the task.
    if(xTaskCreate(SPITask, (const portCHAR *)"SPI", SPITASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SPI_TASK, NULL) != pdTRUE) 
    {
        return(1);
    }
    // Success.
    return(0);
}

