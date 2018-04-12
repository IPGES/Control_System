//*****************************************************************************
//
// gpio_task.c - Task for switching breaker.
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
#include "gpio_task.h"

#include "sysctl.h" //for init ports
#include "gpio.h"

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
#define GPIOSTASKSTACKSIZE        128         // Stack size in words

extern xSemaphoreHandle g_pUARTSemaphore;

void GPIO_Heartbeat_set_high(void);
void GPIO_Heartbeat_set_low(void);
//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************

static void GPIOTask(void *pvParameters)
{
	portTickType ui32WakeTime;
	
	xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
	UARTprintf("GPIO Init\n");
	xSemaphoreGive(g_pUARTSemaphore);

	// Get the current tick count.
	ui32WakeTime = xTaskGetTickCount();
	
	// Loop forever.
	while(1)
	{  
		GPIO_Heartbeat_set_high();
		//GPIO_CAP1_set_high();
		//GPIO_CAP2_set_high();
		//GPIO_CAP3_set_high();
		//GPIO_Breaker_set_high();
		vTaskDelayUntil(&ui32WakeTime, 1000 / portTICK_RATE_MS); // Sleep Scheduler
		GPIO_Heartbeat_set_low();
		//GPIO_CAP1_set_low();
		//GPIO_CAP2_set_low();
		//GPIO_CAP3_set_low();
		//GPIO_Breaker_set_low();
		vTaskDelayUntil(&ui32WakeTime, 1000 / portTICK_RATE_MS); // Sleep Scheduler
	}  //forever loop 
}

void GPIO_CAP1_set_high(void) {
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
}
void GPIO_CAP1_set_low(void) {
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00);
}
void GPIO_CAP2_set_high(void) {
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
}
void GPIO_CAP2_set_low(void) {
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0x00);
}
void GPIO_CAP3_set_high(void) {
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
}
void GPIO_CAP3_set_low(void) {
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00);
}
void GPIO_Breaker_set_high(void) {
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
}
void GPIO_Breaker_set_low(void) {
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x00);
}
void GPIO_Heartbeat_set_high(void) {
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
}
void GPIO_Heartbeat_set_low(void) {
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
}

//*****************************************************************************
//
// Initializes the PWM task to output a PWM to PB6 and it's complement to PB7.
//
//*****************************************************************************
uint32_t GPIOTaskInit(void)
{
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7 | GPIO_PIN_6);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5 | GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
	
    // Create the task.
    if(xTaskCreate(GPIOTask, (const portCHAR *)"GPIO", GPIOSTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_GPIO_TASK, NULL) != pdTRUE) 
    {
        return(1);
    }
    // Success.
    return(0);
}

