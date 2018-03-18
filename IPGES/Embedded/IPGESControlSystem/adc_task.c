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
#include "adc_task.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "adc.h" //for adc lib
#include "hw_memmap.h" //for address bases
#include "sysctl.h" //for init ports
#include "gpio.h" //for gpio to be interfaced to adc
#include "timer.h" //for timer
#include "interrupt.h" //for interrupt
#include "hw_ints.h" //for INT_TIMER2A
//#include "heap_1.h"

//*****************************************************************************
//
// The stack size for the task.
//
//*****************************************************************************
#define ADCTASKSTACKSIZE        128         // Stack size in words

//*****************************************************************************
//
// ADC Init Macros
//
//*****************************************************************************
#define ADC_SEQUENCE2           2
#define ADC_SEQUENCE2_PRIORITY  2

//*****************************************************************************
//
// The item size and queue size for the message queue.
//
//*****************************************************************************
#define ADC_ITEM_SIZE           sizeof(uint8_t)
#define ADC_QUEUE_SIZE          5

//*****************************************************************************
//
// The queue that holds messages sent to the LED task.
//
//*****************************************************************************
xQueueHandle g_pAdcQueue;

extern xSemaphoreHandle g_pUARTSemaphore;
//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//

//*****************************************************************************
void (*ProducerTask)(AdcData_t pDataStruct);  
void ADC0Seq2_Handler(void);
void Timer2IntHandler(void);
void Timer0IntHandler(void);
void clearAdcData (AdcData_t *data);
void setAdcData (AdcData_t *data);

#define SAMPLES_PER_SEC 1000
#define ARRAY_SIZE 500 //250
AdcData_t *adcRawInput;
uint16_t adc_input_index = 0;

static AdcData_t max;
static AdcData_t min;
static AdcData_t rms;

xSemaphoreHandle arrayFull;

static uint16_t prevPosition = 0;
static uint16_t currPosition = 0;
static uint16_t zeroCount = 0;
#define ZERO_POINT 1997
static uint16_t AC_freq = 0;

unsigned long adcCount = 0; //debug

static void ADCTask(void *pvParameters)
{
		// Print the current loggling LED and frequency.
		xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("ADC Init\n");
		xSemaphoreGive(g_pUARTSemaphore);
    portTickType ui32WakeTime;

    // Get the current tick count.
    ui32WakeTime = xTaskGetTickCount();
		int timeOut = 0xffff;
		int result;
		int sum;
	
		while(1) {
			 if( xSemaphoreTake( arrayFull, timeOut ) == pdTRUE ) {
					if(adcRawInput[0].PE0 < ZERO_POINT) {
						prevPosition = 0;
					} else {
						prevPosition = 1;
					}
					for(int i = 1; i < ARRAY_SIZE; i++) {
						if(adcRawInput[i].PE0 < ZERO_POINT) { //get current position
							currPosition = 0;
						} else {
							currPosition = 1;
						}
						if(prevPosition != currPosition) {
							zeroCount++;
						}
						prevPosition = currPosition;
					}
					AC_freq = zeroCount;
					//UARTprintf("Freq %d\n", zeroCount );
					zeroCount = 0;
				}
				result = (sum * 3300) / 4095;
				//UARTprintf("RMS Voltage %d, ", (result * 62876/ 10000) );
		}			 
}

int sqrt(int input) {
	
		return 0;
    // Loop forever.
    /*while(1)
    {  
        if( xSemaphoreTake( arrayFull, timeOut ) == pdTRUE ) //value of 0 used for polling
        {
					
					for(int i = 0; i < ARRAY_SIZE; i++) {
						if(adcRawInput[i].PE0 > max.PE0) {
							max.PE0 = adcRawInput[i].PE0;
						}
						if(adcRawInput[i].PE0 < min.PE0) {
							min.PE0 = adcRawInput[i].PE0;
						}
						if(adcRawInput[i].PE1 > max.PE1) {
							max.PE1 = adcRawInput[i].PE1;
						}
						if(adcRawInput[i].PE1 < min.PE1) {
							min.PE1 = adcRawInput[i].PE1;
						}
						if(adcRawInput[i].PE2 > max.PE2) {
							max.PE2 = adcRawInput[i].PE2;
						}
						if(adcRawInput[i].PE2 < min.PE2) {
							min.PE2 = adcRawInput[i].PE2;
						}
						if(adcRawInput[i].PE3 > max.PE3) {
							max.PE3 = adcRawInput[i].PE3;
						}
						if(adcRawInput[i].PE3 < min.PE3) {
							min.PE3 = adcRawInput[i].PE3;
						}
					}
					
					
					max.PE0 = (max.PE0 * 3300)/4095;
					max.PE1 = (max.PE1 * 3300)/4095;
					max.PE2 = (max.PE2 * 3300)/4095;					
					max.PE3 = (max.PE3 * 3300)/4095;
					
					min.PE0 = (min.PE0 * 3300)/4095;
					min.PE1 = (min.PE1 * 3300)/4095;
					min.PE2 = (min.PE2 * 3300)/4095;					
					min.PE3 = (min.PE3 * 3300)/4095;
					
					
					rms.PE0 = (((max.PE0 - min.PE0) * 50)/141);  //fixed point decimal calculations since floating point kills CPU time. We are approximating anyway since our signal conditioning boards don't do RMS
					rms.PE1 = (((max.PE1 - min.PE1) * 50)/141); 
					rms.PE2 = (((max.PE2 - min.PE2) * 50)/141); 
					rms.PE3 = (((max.PE3 - min.PE3) * 50)/141); 
					
					
					rms.PE0 = (max.PE0 - min.PE0);  //fixed point decimal calculations since floating point kills CPU time. We are approximating anyway since our signal conditioning boards don't do RMS
					rms.PE1 = (max.PE1 - min.PE1); 
					rms.PE2 = (max.PE2 - min.PE2); 
					rms.PE3 = (max.PE3 - min.PE3);
					
					//ADC_Print();
					ADC_PrintJSON();
					setAdcData(&min);
					clearAdcData(&max);
					//xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
					//UARTprintf("adcCount: %d\n", adcCount);
					//xSemaphoreGive(g_pUARTSemaphore);
        }
    } //forever loop */
}

void ADC_Print(void) {
	xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
	UARTprintf("Max PE3: %d   |   PE2: %d   |    PE1: %d    |   PE0: %d\n", max.PE3, max.PE2, max.PE1, max.PE0);
	UARTprintf("Min PE3: %d   |   PE2: %d   |    PE1: %d    |   PE0: %d\n", min.PE3, min.PE2, min.PE1, min.PE0);
	xSemaphoreGive(g_pUARTSemaphore);
}

void ADC_PrintJSON(void) {
	xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
	UARTprintf("@{\"pv\" : %d, \"inverter\" : %d, \"wind\" : %d, \"grid\" : %d, \"load\" : %d,}\n", rms.PE0, rms.PE1, rms.PE2, rms.PE3, 419);
	xSemaphoreGive(g_pUARTSemaphore);
}

uint16_t ADC_PrintFreq(void) {
	return AC_freq;
}

//*****************************************************************************
//
// Initializes the LED task.
//
//*****************************************************************************
uint32_t ADCTaskInit(void(*pTask)(AdcData_t pDataStruct))
{

		ProducerTask = pTask;
	
    // Create a queue for sending messages to the LED task.
    //g_pLEDQueue = xQueueCreate(ADC_QUEUE_SIZE, ADC_ITEM_SIZE);

    /* //working timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    IntMasterEnable();
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet());
    IntEnable(INT_TIMER2A);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    IntRegister(INT_TIMER2A, &Timer2IntHandler);
     TimerEnable(TIMER2_BASE, TIMER_A);
     */
       
    //IntMasterEnable(); //needed? Should be non critical
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0); //must have enabled ADC first
    TimerDisable(TIMER2_BASE, TIMER_A);
    TimerControlTrigger(TIMER2_BASE, TIMER_A, true); //enable timer2A trigger to ADC
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // Disables the timers, but doesn't enable again
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/SAMPLES_PER_SEC); //SysCltClockGet returns count for 1 second so SysCtlClockGet() / 1000 sets the Timer0B load value to 1ms.
		TimerIntDisable(TIMER2_BASE, 0xFFFFFFFF ); //disable all interrupts for this timer
    TimerEnable(TIMER2_BASE, TIMER_A);
		
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_EIGHTH , 1); //last param is divider
    ADCSequenceDisable(ADC0_BASE, ADC_SEQUENCE2); 
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE2, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 0, ADC_CTL_CH3 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 1, ADC_CTL_CH2 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 2, ADC_CTL_CH1 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 3, ADC_CTL_CH0 | ADC_CTL_END | ADC_CTL_IE); //adc_base, Sequence Number, Step, set flag and end after first
    ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE2); //adc_base, sequence 
    ADCIntEnable(ADC0_BASE, ADC_SEQUENCE2);
    ADCIntRegister(ADC0_BASE, ADC_SEQUENCE2, &ADC0Seq2_Handler);
    IntPrioritySet(INT_ADC0SS2, ADC_SEQUENCE2_PRIORITY);
    IntEnable(INT_ADC0SS2);
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE2);
		
		arrayFull = xSemaphoreCreateBinary();
		
		setAdcData(&min);
		clearAdcData(&max);
		adcRawInput = pvPortMalloc(1000 * sizeof(AdcData_t));
		
    // Create the task.
    if(xTaskCreate(ADCTask, (const portCHAR *)"ADC", ADCTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_ADC_TASK, NULL) != pdTRUE) 
    {
        return(1);
    }
    // Success.
    return(0);
}

void clearAdcData (AdcData_t *data) {
	data->PE0 = 0;
	data->PE1 = 0;
	data->PE2 = 0;
	data->PE3 = 0;
}
void setAdcData (AdcData_t *data) {
	data->PE0 = 4095;
	data->PE1 = 4095;
	data->PE2 = 4095;
	data->PE3 = 4095;
}

void ADC0Seq2_Handler(void)
{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE2); // Clear the timer interrupt flag.

    ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE0);
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE1);
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE2);
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawInput[adc_input_index].PE3);
	
		adcCount++;
		adc_input_index = (adc_input_index + 1) % (ARRAY_SIZE);
		if(adc_input_index == (ARRAY_SIZE - 1)) {
			xSemaphoreGiveFromISR(arrayFull, &xHigherPriorityTaskWoken);
		}
}
