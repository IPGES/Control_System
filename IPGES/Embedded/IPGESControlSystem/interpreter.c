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
#include "interpreter.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "adc_task.h"
#include "pwm_task.h"
#include "spi_task.h"
#include "gpio_task.h"

//*****************************************************************************
//
// The stack size for the task.
//
//*****************************************************************************
#define INTERPRETERTASKSTACKSIZE        128         // Stack size in words

//*****************************************************************************
//
// ADC Init Macros
//
//*****************************************************************************
#define INPUTLENGTH           20 //includes null

//*****************************************************************************
//
// The item size and queue size for the message queue.
//
//*****************************************************************************
#define INTERPRETER_ITEM_SIZE           sizeof(uint8_t)
#define INTERPRETER_QUEUE_SIZE          5

//*****************************************************************************
//
// The queue that holds messages sent to the LED task.
//
//*****************************************************************************
xQueueHandle g_pInterpreterQueue;

extern xSemaphoreHandle g_pUARTSemaphore;
//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************

static void InterpreterTask(void *pvParameters)
{
		portTickType ui32WakeTime;
    
		char uartInput[INPUTLENGTH];
	
		int loadDutyCycle;
		int windDutyCycle;
		int gpio_level;
		int gpio_port; 
		int pvValue;
		int breakerValue; 

		xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
		UARTprintf("Interpreter Init\n");
		xSemaphoreGive(g_pUARTSemaphore);
	
    ui32WakeTime = xTaskGetTickCount(); // Get the current tick count.
	
    while(1)
    {  
			UARTgets(uartInput, INPUTLENGTH); //note this is blocking, use peek to do non-blocking
			switch(uartInput[0]) {
				case 'A': //ADC
					ADC_Print();
					break;
				case 'B': //Breaker
					breakerValue = (uartInput[8] - 48);
					if(breakerValue == 0) {
						GPIO_Breaker_set_low();
					} else {
						GPIO_Breaker_set_high();
					}
					break;
				case 'G': 
					gpio_port = (uartInput[4] - 48);
					gpio_level = (uartInput[6] - 48); 
					if(gpio_port == 1) {
						if(gpio_level == 1) {
							GPIO_CAP1_set_high();
						} else {
							GPIO_CAP1_set_low(); 
						}
					}
					if(gpio_port == 2) {
						if(gpio_level == 1) {
							GPIO_CAP2_set_high();
						} else {
							GPIO_CAP2_set_low(); 
						}
					}
					if(gpio_port ==3) {
						if(gpio_level == 1) {
							GPIO_CAP3_set_high();
						} else {
							GPIO_CAP3_set_low(); 
						}
					}
					break; 
				case 'L': //Load 010
					loadDutyCycle = (uartInput[5] - 48) * 100 + (uartInput[6] - 48) * 10 + uartInput[7] - 48;
					if(0 <= loadDutyCycle && loadDutyCycle <= 100) {
						PWM_change_duty_chopper(loadDutyCycle);
					} else {
						xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
						UARTprintf("Duty cycle must be between 0-100 inclusive and three digits (024 = 24).\n");
						xSemaphoreGive(g_pUARTSemaphore);
					}
					break;
				case 'P': //PV 099
					pvValue = (uartInput[3] - 48) * 100 + (uartInput[4] - 48) * 10 + uartInput[5] - 48;
					if(0 <= pvValue && pvValue <= 128) {
						SPI_change_output(pvValue);
					} else {	
						xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
						UARTprintf("PV must be between 0-128 inclusive and three digits (024 = 24).\n");
						xSemaphoreGive(g_pUARTSemaphore);
					}
					break;
				case 'W': //Wind 099
					windDutyCycle = (uartInput[5] - 48) * 100 + (uartInput[6] - 48) * 10 + uartInput[7] - 48;
					if(0 <= windDutyCycle && windDutyCycle <= 100) {
						PWM_change_duty_wind(windDutyCycle);
					} else {	
						xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
						UARTprintf("Duty cycle must be between 0-100 inclusive and three digits (024 = 24).\n");
						xSemaphoreGive(g_pUARTSemaphore);
					}
					break;
				default :
					xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
					UARTprintf("Command not found\n");
					xSemaphoreGive(g_pUARTSemaphore);
			}
		vTaskDelayUntil(&ui32WakeTime, 1000 / portTICK_RATE_MS);
	}
}
//		SPI_change_output(output);
//*****************************************************************************
//
// Initializes the LED task.
//
//*****************************************************************************
uint32_t InterpreterTaskInit(void)
{
    // Create a queue for sending messages to the LED task.
    //g_pLEDQueue = xQueueCreate(ADC_QUEUE_SIZE, ADC_ITEM_SIZE);

    // Create the task.
    if(xTaskCreate(InterpreterTask, (const portCHAR *)"Interpreter", INTERPRETERTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_INTERPRETER_TASK, NULL) != pdTRUE) 
    {
        return(1);
    }
		//Success
    return(0);
}
