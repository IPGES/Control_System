//*****************************************************************************
//
// adc_task.c - A simple flashing LED task.
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
#define ADC_SEQUENCE0           0
//#define ADC_SEQUENCE2           2

//#define ADC_SEQUENCE2_PRIORITY  1
#define ADC_SEQUENCE0_PRIORITY  0

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
void (*ProducerTask)();  
//void ADC0Seq2_Handler(void);
void ADC0Seq0_Handler(void);
//void Timer2IntHandler(void);
//void Timer0IntHandler(void);
void clearAdcData (AdcSS0Data *data);
void setAdcData (AdcSS0Data *data);
int sqrt(int input);
int undo_signal_conditioning_load_vrms(int input);

#define scb_mean 2022
#define scb_factor 11
#define transformer_factor 5

#define SAMPLES_PER_SEC 1000 //sampling too many times causes contention between the two sequencers
#define ARRAY_SIZE 250 //250
AdcSS0Data *adcRawSS0Input;
//AdcSS2Data *adcRawSS2Input;
//uint16_t adc_ss2_index = 0;
uint16_t adc_ss0_index = 0;
static uint32_t v_rms;

//xSemaphoreHandle ss2Full;
xSemaphoreHandle ss0Full;

unsigned long adcCount = 0; //debug

static void ADCTask(void *pvParameters)
{
		int timeOut = 0xffff;
		int result = 0;
		int sum = 0;;
	
		// Print the current loggling LED and frequency.
		xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("ADC Init\n");
		xSemaphoreGive(g_pUARTSemaphore);

		while(1) {
       if( xSemaphoreTake( ss0Full, timeOut ) == pdTRUE ) {
          for(int i = 0; i < ARRAY_SIZE; i++) {
						//UARTprintf("%d, ", (adcRawSS0Input[i].PE0 * 3300) / (4095) );
						int shifted_adc = adcRawSS0Input[i].PE0 - scb_mean; //fix point calculations
						//UARTprintf("%d, ", (shifted_adc * 3300) / 4095);
						
						sum += shifted_adc * shifted_adc;
						
						//sum += (adcRawSS0Input[i].PE0 * 3300) / 4095; //average
						//UARTprintf("%d, ", undo_signal_conditioning_load_vrms(shifted_adc) );
						
          }
          sum /= ARRAY_SIZE;
          sum = sqrt(sum);
					result = ((sum) * 3300) / 4095;
					//v_rms = result;
					v_rms = result;
					//v_rms = undo_signal_conditioning_load_vrms(result);
					/*
					xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
					UARTprintf("RMS Voltage %d, ", undo_signal_conditioning_load_vrms(result) );
					xSemaphoreGive(g_pUARTSemaphore);*/
        }
    }
}

int undo_signal_conditioning_load_vrms(int input) {
	int result;
	if(input < 36) { 
		result = (input * 117) /100; 
	} else if (input < 52) {
		result = (input * 2423) /100; 
	} else if (input < 65) {
		result = (input * 3169) /100; 
	} else if (input < 78) {
		result = (input * 3705) /100; 
	} else if (input < 94) {
		result = (input * 4074) /100; 
	} else if (input < 112) {
		result = (input * 4258) /100; 
	} else if (input < 131) {
		result = (input * 4404) /100; 
	} else if (input < 147) {
		result = (input * 4605) /100; 
	} else if (input < 166) {
		result = (input * 4645) /100; 
	} else if (input < 186) {
		result = (input * 4677) /100; 
	} else if (input < 202) {
		result = (input * 4772) /100; 
	} else if (input < 221) {
		result = (input * 4787) /100; 
	} else if (input < 237) {
		result = (input * 4831) /100; 
	} else if (input < 257) {
		result = (input * 4832) /100; 
	} else if (input < 273) {
		result = (input * 4875) /100; 
	} else if (input < 286) {
		result = (input * 4961) /100; 
	} else if (input < 302) {
		result = (input * 4967) /100; 
	} else if (input < 319) {
		result = (input * 4978) /100; 
	} else {
		result = (input * 4956) / 100;
	} 
	return result;   
}

int sqrt(int input) {
  
  int guess = 1050;
  int guess_sq = 1102500;
  int delta = 525;
  for(int i = 0; i < 9; i++) {
    if(input > guess_sq) {
      guess += delta;
      guess_sq = (guess) * (guess);
    } else {
      guess -= delta;
      guess_sq = (guess) * (guess);
    }
    delta /= 2;
  }
  return guess; 
} 

void ADC_Print(void) {
	xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
	//UARTprintf("PE3: %d   |   PE2: %d   |    PE1: %d    |   PE0: %d\n", adcRawSS0Input[110].PE3, adcRawSS0Input[10].PE2, adcRawSS0Input[20].PE1, adcRawSS0Input[0].PE0);
	//UARTprintf("PD3: %d   |   PD2: %d   |    PE5: %d    |   PE4: %d\n", adcRawSS0Input[110].PD3, adcRawSS0Input[10].PD2, adcRawSS0Input[20].PE5, adcRawSS0Input[0].PE4);
	xSemaphoreGive(g_pUARTSemaphore);
}

void ADC_PrintJSON(void) {
	xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
	//UARTprintf("@{\"pv\" : %d, \"inverter\" : %d, \"wind\" : %d, \"grid\" : %d, \"load\" : %d,}\n", 0, 0, 0, 0, v_rms);
	UARTprintf("@{grid: \"load\" : %d,}\n", v_rms);
	//UARTprintf("@{grid: \"load\" : %d,}\n", undo_signal_conditioning_load_vrms(v_rms));
	xSemaphoreGive(g_pUARTSemaphore);
}

//*****************************************************************************
//
// Initializes the LED task.
//
//*****************************************************************************
uint32_t ADCTaskInit(void(*pTask))
{
	
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
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0); //must have enabled ADC first
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0); //must have enabled ADC first
		
		// timer 2 associated with ADC 0
		TimerDisable(TIMER2_BASE, TIMER_A);
    TimerControlTrigger(TIMER2_BASE, TIMER_A, true); //enable timer2A trigger to ADC
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // Disables the timers, but doesn't enable again
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/SAMPLES_PER_SEC); //SysCltClockGet returns count for 1 second so SysCtlClockGet() / 1000 sets the Timer0B load value to 1ms.
		TimerIntDisable(TIMER2_BASE, 0xFFFFFFFF ); //disable all interrupts for this timer
    TimerEnable(TIMER2_BASE, TIMER_A);  
		
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_EIGHTH , 1); //last param is divider
    ADCHardwareOversampleConfigure(ADC0_BASE, 64);
		
		//sequencer 0
		ADCSequenceDisable(ADC0_BASE, ADC_SEQUENCE0); 
		ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE0, ADC_TRIGGER_TIMER, ADC_SEQUENCE0_PRIORITY);
		ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE0, 0, ADC_CTL_CH0 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE0, 1, ADC_CTL_CH1 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE0, 2, ADC_CTL_CH2 );
		ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE0, 3, ADC_CTL_CH3 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE0, 4, ADC_CTL_CH4 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE0, 5, ADC_CTL_CH5 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE0, 6, ADC_CTL_CH8 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE0, 7, ADC_CTL_CH9 | ADC_CTL_END | ADC_CTL_IE); //adc_base, Sequence Number, Step, set flag and end after first
		ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE0); //adc_base, sequence 
		ADCIntEnable(ADC0_BASE, ADC_SEQUENCE0);
		ADCIntRegister(ADC0_BASE, ADC_SEQUENCE0, &ADC0Seq0_Handler);
		IntPrioritySet(INT_ADC0SS0, 0);
    IntEnable(INT_ADC0SS0);
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE0);	
		
		/*
		// sequencer 2 Cannot use since sampling rate goes to the trash
		ADCSequenceDisable(ADC0_BASE, ADC_SEQUENCE2); 
		ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE2, ADC_TRIGGER_TIMER, ADC_SEQUENCE2_PRIORITY);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 0, ADC_CTL_CH3 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 1, ADC_CTL_CH2 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 2, ADC_CTL_CH1 );
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE2, 3, ADC_CTL_CH0 | ADC_CTL_END | ADC_CTL_IE); //adc_base, Sequence Number, Step, set flag and end after first
		ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE2); //adc_base, sequence 
		ADCIntEnable(ADC0_BASE, ADC_SEQUENCE2);
		ADCIntRegister(ADC0_BASE, ADC_SEQUENCE2, &ADC0Seq2_Handler);
		IntPrioritySet(INT_ADC0SS2, 0);
    IntEnable(INT_ADC0SS2);
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE2);	*/
		
		//ss2Full = xSemaphoreCreateBinary();
		ss0Full = xSemaphoreCreateBinary();
		
		//adcRawSS2Input = pvPortMalloc(250 * sizeof(AdcSS2Data)); //IMPORTANT FOR DEBUGGING
		adcRawSS0Input = pvPortMalloc(250 * sizeof(AdcSS0Data)); //IMPORTANT FOR DEBUGGING
    
		// Create the task.
    if(xTaskCreate(ADCTask, (const portCHAR *)"ADC", ADCTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_ADC_TASK, NULL) != pdTRUE) 
    {
        return(1);
    }
    // Success.
    return(0);
}

void clearAdcData (AdcSS0Data *data) {
	data->PE0 = 0;
	data->PE1 = 0;
	data->PE2 = 0;
	data->PE3 = 0;
}
void setAdcData (AdcSS0Data *data) {
	data->PE0 = 4095;
	data->PE1 = 4095;
	data->PE2 = 4095;
	data->PE3 = 4095;
}

/*
void ADC0Seq2_Handler(void)
{
		//int test = 0;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE2); // Clear the timer interrupt flag.

    ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawSS2Input[adc_ss2_index].PE3);
		//ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &test);
		//UARTprintf("PD3: %d\n", test);
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawSS2Input[adc_ss2_index].PE2);
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawSS2Input[adc_ss2_index].PE1);
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE2, &adcRawSS2Input[adc_ss2_index].PE0);
		

		adc_ss2_index = (adc_ss2_index + 1) % (ARRAY_SIZE);

		if(adc_ss2_index == (ARRAY_SIZE - 1)) {
			xSemaphoreGiveFromISR(ss2Full, &xHigherPriorityTaskWoken);
		}
	
}
*/



void ADC0Seq0_Handler(void)
{	
		//int test = 0;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE0); // Clear the timer interrupt flag.
	
		ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &adcRawSS0Input[adc_ss0_index].PE3);
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &adcRawSS0Input[adc_ss0_index].PE2);
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &adcRawSS0Input[adc_ss0_index].PE1);
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &adcRawSS0Input[adc_ss0_index].PE0);
		ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &adcRawSS0Input[adc_ss0_index].PD3);
		ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &adcRawSS0Input[adc_ss0_index].PD2);
		ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &adcRawSS0Input[adc_ss0_index].PE5);
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &adcRawSS0Input[adc_ss0_index].PE4);
		
	
		//ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE0, &test);
		//UARTprintf("PD3: %d\n", test);
		
		adc_ss0_index = (adc_ss0_index + 1) % (ARRAY_SIZE);
	
		if(adc_ss0_index == (ARRAY_SIZE - 1)) {
			xSemaphoreGiveFromISR(ss0Full, &xHigherPriorityTaskWoken);
		}
	
}

