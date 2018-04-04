//*****************************************************************************
//
// adc_task.c - ADC
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
void ADC0Seq0_Handler(void);
void clearAdcData (AdcSS0Data *data);
void setAdcData (AdcSS0Data *data);
int sqrt(int input);
int undo_signal_conditioning_load_vrms(int input);
int undo_signal_conditioning_load_irms(int input);
int undo_signal_conditioning_dist_vrms(int input);
int undo_signal_conditioning_dist_irms(int input);
int scaling(int input, int *boundary, int *scale);

#define scb_mean_load_vrms 2022 //Power Supply 4.131
#define scb_mean_load_irms 2891 //
#define scb_mean_dist_vrms 0000 //Power Supply 4.131
#define scb_mean_dist_irms 0000 //
#define scb_factor 11
#define transformer_factor 5

#define SAMPLES_PER_SEC 1000 //sampling too many times causes contention between the two sequencers
#define ARRAY_SIZE 250 //250
AdcSS0Data *adcRawSS0Input;
uint16_t adc_ss0_index = 0;
static uint32_t load_v_rms;
static uint32_t load_i_rms;
static uint32_t dist_v_rms;
static uint32_t dist_i_rms;
//int test; 

xSemaphoreHandle ss0Full;

unsigned long adcCount = 0; //debug

static void ADCTask(void *pvParameters)
{
		int timeOut = 0xffff;
		int result_load_vrms = 0;
		int result_load_irms = 0;
		int result_dist_vrms = 0;
		int result_dist_irms = 0;
		int sum_load_vrms = 0;
		int sum_load_irms = 0;
		int sum_dist_vrms = 0;
		int sum_dist_irms = 0;
		AdcSS0Data shifted_adc;
	
		// Print the current loggling LED and frequency.
		xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("ADC Init\n");
		xSemaphoreGive(g_pUARTSemaphore);

		while(1) {
       if( xSemaphoreTake( ss0Full, timeOut ) == pdTRUE ) {
          for(int i = 0; i < ARRAY_SIZE; i++) {
						//PE0
						shifted_adc.PE0 = adcRawSS0Input[i].PE0 - scb_mean_load_vrms; //5.4 -> 1.63 mean 
						sum_load_vrms += shifted_adc.PE0 * shifted_adc.PE0;
						//PE1
						shifted_adc.PE1 = adcRawSS0Input[i].PE1 - scb_mean_load_irms; //5.4 -> 2.31
						sum_load_irms += shifted_adc.PE1 * shifted_adc.PE1; 
						//PE2
						shifted_adc.PE2 = adcRawSS0Input[i].PE2 - scb_mean_dist_vrms;
						sum_dist_vrms += shifted_adc.PE2 * shifted_adc.PE2;
						//PE3
						shifted_adc.PE3 = adcRawSS0Input[i].PE2 - scb_mean_dist_irms;
						sum_dist_irms += shifted_adc.PE3 * shifted_adc.PE3; 
						
						//UARTprintf("%d,", (adcRawSS0Input[i].PE2* 3300) / 4095);
						//UARTprintf("%d,", adcRawSS0Input[i].PE0);
						//UARTprintf("%d,",shifted_adc.PE0);
          }
          sum_load_vrms /= ARRAY_SIZE;
					sum_load_irms /= ARRAY_SIZE;
					
          sum_load_vrms = sqrt(sum_load_vrms);
					sum_load_irms = sqrt(sum_load_irms);
					
					result_load_vrms = ((sum_load_vrms) * 3300) / 4095;
					result_load_irms = ((sum_load_irms) * 3300) / 4095;
					
					load_v_rms = undo_signal_conditioning_load_vrms(result_load_vrms);
			 		load_i_rms = result_load_irms;	
					
        }
    }
}

int undo_signal_conditioning_load_vrms(int input) {
	int boundaries [28] = {20, 23, 26, 29, 36, 39, 45, 52, 55, 62, 68, 74, 84, 91, 97, 108, 137, 157, 173, 192, 215, 224, 244, 263, 279, 295, 308, 326};
	int scale [28]  = {65,465,2265,3300,3611,4169,4340,4396,4778,4800,4929,5064,4904,5010,5134,5564,5145,5133,5225,5140,5130,5281,5241,5197,5211,5220,5211,5220};
	int real_value = scaling(input, boundaries, scale);
	return real_value;   
}

int undo_signal_conditioning_load_irms(int input) {
	int boundaries [28] = {58,62,65,65,68,68,71,71,74,78,81,87,91,97,103,124,153,182,221,263,323,384,440,503,563,624,660,685};
	int scale [28]  = {440, 490, 470, 490, 520, 550, 580, 610, 640, 670, 715, 760, 820, 865, 930, 1120, 1330, 1560, 1840, 2140, 2510, 2860, 3230, 3620, 4045, 4490, 4970, 5530};
	int real_value = scaling(input, boundaries, scale);
	return real_value;   
}

int undo_signal_conditioning_dist_vrms(int input) {
	int boundaries [28] = {20, 23, 26, 29, 36, 39, 45, 52, 55, 62, 68, 74, 84, 91, 97, 108, 137, 157, 173, 192, 215, 224, 244, 263, 279, 295, 308, 326};
	int scale [28]  = {65,465,2265,3300,3611,4169,4340,4396,4778,4800,4929,5064,4904,5010,5134,5564,5145,5133,5225,5140,5130,5281,5241,5197,5211,5220,5211,5220};
	int real_value = scaling(input, boundaries, scale);
	return real_value;   
}

int undo_signal_conditioning_dist_irms(int input) {
	int boundaries [28] = {58,62,65,65,68,68,71,71,74,78,81,87,91,97,103,124,153,182,221,263,323,384,440,503,563,624,660,685};
	int scale [28]  = {440, 490, 470, 490, 520, 550, 580, 610, 640, 670, 715, 760, 820, 865, 930, 1120, 1330, 1560, 1840, 2140, 2510, 2860, 3230, 3620, 4045, 4490, 4970, 5530};
	int real_value = scaling(input, boundaries, scale);
	return real_value;   
}

int scaling(int input, int *boundary, int *scale) {
	int result = 0;
	int low_numerator, high_numerator, difference = 0;
	//int test = 0; 
	if(input < boundary[0]) {
		result = (input * scale[0]) / 100;
	}
	
	for(int i = 1; i < 28; i++) {
		if(input >= boundary[i-1] && input < boundary[i]) {
			result = (input * ((scale[i-1] + scale[i])/2)) / 100; 
			low_numerator = (input - boundary[i-1]);
			high_numerator = ((boundary[i] - input));
			difference = boundary[i] - boundary[i-1]; 
			result = ((low_numerator * scale[i-1]) + (high_numerator * scale[i])) / difference;
			//test = ((scale[i-1] + scale[i])/2);
		}			
	}
	if(input >= boundary[27]) {
		result = (input * scale[27]) / 100;
		//test = 27;
	}
	
	return result;
}	

int sqrt(int input) {
  
  int guess = 2100;
  int guess_sq = guess * guess;
  int delta = 1050;	
  for(int i = 0; i < 10; i++) {
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
	//UARTprintf("@{\"pv\" : %d, \"inverter\" : %d, \"wind\" : %d, \"grid\" : %d, \"load\" : %d,}\n", 0, 0, 0, 0, load_v_rms);
	//UARTprintf("@{grid: \"load\" : %d}\n", (load_i_rms * load_v_rms)/1000);
	UARTprintf("@{grid: \"load\" : %d}\n", load_i_rms);
	//UARTprintf("@{grid: \"load\" : %d, test %d}\n", load_v_rms, test);
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
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0); //must have enabled ADC first
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_2 ); //must have enabled ADC first
		
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

