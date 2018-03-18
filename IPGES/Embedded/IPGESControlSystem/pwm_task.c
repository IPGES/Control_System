//*****************************************************************************
//
// pwm_task.c - PWM for variable load chopper, and wind.
//
// PB4 (PWM1 - Wind)
// PB6 (PWM0 - Load)
// PB7 (PWM0 - Load)
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "pwm_task.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "pwm.h" //for pwm lib
#include "hw_memmap.h" //for address bases
#include "sysctl.h" //for init ports
#include "gpio.h" //for gpio to be interfaced to adc
#include "interrupt.h" //for interrupt
#include "hw_ints.h" //for INT_TIMER2A
#include "pin_map.h" //for GPIO_PB6_M0PWM0

#define GPIO_PORTB_DR8R_R       (*((volatile uint32_t *)0x40005508))
//*****************************************************************************
//
// The stack size for the task.
//
//*****************************************************************************
#define PWMTASKSTACKSIZE        128         // Stack size in words

//*****************************************************************************
//
// ADC Init Macros
//
//*****************************************************************************
#define ADC_SEQUENCE2           2
#define ADC_SEQUENCE2_PRIORITY  2

//*****************************************************************************
//
// The item size and queue size for the message queue. (Not Used)
//
//*****************************************************************************
#define PWM_ITEM_SIZE           sizeof(uint8_t)
#define PWM_QUEUE_SIZE          5

//*****************************************************************************
//
// The queue that holds messages sent to the LED task.
//
//*****************************************************************************
xQueueHandle g_pPwmQueue;

extern xSemaphoreHandle g_pUARTSemaphore;
//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************

static void PWMTask(void *pvParameters)
{
		portTickType ui32WakeTime;
		
		xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
		UARTprintf("PWM Init\n");
		xSemaphoreGive(g_pUARTSemaphore);

    // Get the current tick count, used for vTaskDelayUntil();
    ui32WakeTime = xTaskGetTickCount();

    // Loop forever.
    while(1)
    {  
			vTaskDelayUntil(&ui32WakeTime, 1000 / portTICK_RATE_MS); // Scheduler Sleeps
    } //forever loop
}

//*****************************************************************************
//
// Initializes the PWM0 to output to PB6 and it's complement to PB7 
// for load. Then PWM1 to output to PB4 for wind.
//
//*****************************************************************************
uint32_t PWMTaskInit(void)
{
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1); //set PWM clock to processor clock with multiplier of 1
		SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DB_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 2500);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 2500* 50/100);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMOutputInvert(PWM0_BASE, PWM_OUT_0_BIT, false);	//AC Chopper takes a PWM signal and it's complement.
    GPIO_PORTB_DR8R_R |=0xC0; //The chopper driver must have 8mA output
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
 
		//SysCtlPWMClockSet(SYSCTL_PWMDIV_1); //set PWM clock to processor clock with multiplier of 1
		//SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DB_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 2500);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 2500* 50/100);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
    //PWMOutputInvert(PWM0_BASE, PWM_OUT_1_BIT, true);
    PWMDeadBandEnable(PWM0_BASE, PWM_GEN_0, 0xF, 0xF);
    GPIO_PORTB_DR8R_R |=0xC0; //The chopper driver must have 8mA output
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
		
		GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DB_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 2500);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 2500* 50/100);
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    //PWMOutputInvert(PWM0_BASE, PWM_OUT_1_BIT, true);
    PWMDeadBandEnable(PWM0_BASE, PWM_GEN_1, 0xF, 0xF);
    GPIO_PORTB_DR8R_R |=0xC0; // 8mA output
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	
    /* Used for more complex signals
    PWMIntEnable(PWM0_BASE, PWM_INT_GEN_0); 
    IntMasterEnable();
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);
    IntEnable(INT_PWM0_0);
     */

    // Create the task.
    if(xTaskCreate(PWMTask, (const portCHAR *)"PWM", PWMTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_PWM_TASK, NULL) != pdTRUE) 
    {
        return(1);
    }
    // Success.
    return(0);
}

void PWM_change_duty_chopper(int dutyCycle) {
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 2500* dutyCycle/100);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 2500* dutyCycle/100);
}

void PWM_change_duty_wind(int dutyCycle) {
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 2500* dutyCycle/100);
}

/*
void PWM0IntHandler(void)
{

    PWMGenIntClear(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);
    //
    // Clear the timer interrupt flag.
    //
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 64000/2);

}
*/
