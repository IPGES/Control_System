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

#define scb_factor 11
#define transformer_factor 5

#define SAMPLES_PER_SEC 1000 //sampling too many times causes contention between the two sequencers
#define ARRAY_SIZE 250 //250
AdcSS0Data *adcRawSS0Input;
uint16_t adc_ss0_index = 0;
uint32_t load_v_rms;
uint32_t load_i_rms;
uint32_t dist_v_rms;
uint32_t dist_i_rms;
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
		
		int avg_sum_load_vrms;
		int avg_sum_load_irms;
		int avg_sum_dist_vrms;
		int avg_sum_dist_irms; 
		int scb_mean_load_vrms; //2022 //Power Supply 4.131
		int scb_mean_load_irms; //2891 //
		int scb_mean_dist_vrms; //0000 //Power Supply 4.131
		int scb_mean_dist_irms; //0000 //
	
		// Print the current loggling LED and frequency.
		xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("ADC Init\n");
		xSemaphoreGive(g_pUARTSemaphore);

		while(1) {
			if( xSemaphoreTake( ss0Full, timeOut ) == pdTRUE ) {
					avg_sum_load_vrms = 0;
					avg_sum_load_irms = 0;
					avg_sum_dist_vrms = 0;
					avg_sum_dist_irms = 0;
					for(int i = 0; i < ARRAY_SIZE; i++) {
						avg_sum_load_vrms += adcRawSS0Input[i].PE0;
						avg_sum_load_irms += adcRawSS0Input[i].PE1;
						avg_sum_dist_vrms += adcRawSS0Input[i].PE2;
						avg_sum_dist_irms += adcRawSS0Input[i].PE3;
					//UARTprintf("%d,",adcRawSS0Input[i].PE0);
					}
					scb_mean_load_vrms = avg_sum_load_vrms/ARRAY_SIZE;
					scb_mean_load_irms = avg_sum_load_irms/ARRAY_SIZE;
					scb_mean_dist_vrms = avg_sum_dist_vrms/ARRAY_SIZE;
					scb_mean_dist_irms = avg_sum_dist_irms/ARRAY_SIZE; 
					//UARTprintf("AVG: %d\n",scb_mean_dist_irms);
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
						shifted_adc.PE3 = adcRawSS0Input[i].PE3 - scb_mean_dist_irms;
						sum_dist_irms += shifted_adc.PE3 * shifted_adc.PE3; 
						
						//UARTprintf("%d,", (adcRawSS0Input[i].PE2* 3300) / 4095);
						//UARTprintf("%d,", adcRawSS0Input[i].PE0);
						//UARTprintf("%d,",shifted_adc.PE0);
          }
          sum_load_vrms /= ARRAY_SIZE;
					sum_load_irms /= ARRAY_SIZE;
					sum_dist_vrms /= ARRAY_SIZE;
					sum_dist_irms /= ARRAY_SIZE;
					
          sum_load_vrms = sqrt(sum_load_vrms);
					sum_load_irms = sqrt(sum_load_irms);
					sum_dist_vrms = sqrt(sum_dist_vrms); 
					sum_dist_irms = sqrt(sum_dist_irms);
					
					result_load_vrms = ((sum_load_vrms) * 3300) / 4095;
					result_load_irms = ((sum_load_irms) * 3300) / 4095;
					result_dist_vrms = ((sum_dist_vrms) * 3300) / 4095;
					result_dist_irms = ((sum_dist_irms) * 3300) / 4095;
					
					load_v_rms = undo_signal_conditioning_load_vrms(result_load_vrms);
			 		load_i_rms = undo_signal_conditioning_load_irms(result_load_irms);
					dist_v_rms = undo_signal_conditioning_dist_vrms(result_dist_vrms);
					dist_i_rms = undo_signal_conditioning_load_irms(result_dist_irms);
					
					//UARTprintf("Volt: %d\n", undo_signal_conditioning_load_vrms(23));
					//UARTprintf("Curr: %d\n", result_dist_irms);
					//load_v_rms = result_load_vrms;
					//load_i_rms = result_load_irms;	
					//dist_i_rms
					/*
					UARTprintf("Volt: %d\n", result_dist_vrms);
					UARTprintf("Curr: %d\n", result_dist_irms);
					UARTprintf("Volt: %d\n", result_dist_vrms);
					UARTprintf("Curr: %d\n", result_dist_irms);	
					*/					
					
        }
    }
}

int undo_signal_conditioning_load_vrms(int input) {
	int boundaries [28] = {10,13,20,26,33,36,39,45,52,58,65,71,78,84,94,112,131,147,163,186,202,218,237,257,273,289,308,319};
	int scale [28]  = {1630,1253,2885,3561,3815,4366,4843,4917,4905,4970,4995,5088,5105,5190,5063,5151,5160,5238,5300,5215,5207,5256,5329,5233,5227,5249,5282,5260};
	int real_value = scaling(input, boundaries, scale);
	return real_value;   
}

int undo_signal_conditioning_load_irms(int input) {
	int boundaries [28] = {62,65,68,69,70,71,74,77,78,81,84,91,94,100,108,128,153,186,228,269,319,371,426,484,537,585,618,663};
	int scale [28]  = {790,815,764,768,800,816,810,805,807,827,845,835,851,860,861,867,869,849,828,814,793,778,767,762,761,776,799,844};
	int real_value = scaling(input, boundaries, scale);
	return real_value;   
}

int undo_signal_conditioning_dist_vrms(int input) {
	int boundaries [28] = {7,55,84,103,124,147,166,179,195,218,250,257,289,308,339,371,390,413,432,429,445,455,464,468,477,481,487,497};
	int scale [28]  = {5942,5036,5035,5165,5129,5095,5162,5122,5148,5087,5112,5140,5121,5123,5129,5126,5148,5125,5136,5244,5121,5123,5140,5143,5127,5147,5151,5116};
	int real_value = scaling(input, boundaries, scale);
	return real_value;   
}

int undo_signal_conditioning_dist_irms(int input) {
	int boundaries [28] = {4,7,10,16,23,29,36,42,45,58,68,71,87,100,118,134,147,163,173,182,189,192,199,215,368,474,553,595};
	int scale [28]  = {250,1000,1400,1250,1086,1137,1111,1071,1133,1034,1014,1014,1011,990,974,970,952,926,936,923,915,916,919,897,831,814,848,941};
	int real_value = scaling(input, boundaries, scale);
	return real_value;   
}

int get_load_v_rms() {
	return load_v_rms;
}
int get_load_i_rms() {
	return load_i_rms;
}
int get_dist_v_rms() {
	return dist_v_rms;
}
int get_dist_i_rms() {
	return dist_v_rms;
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
			//result = (input * scale[i-1]) / 100; //round down
			result = (input * ((scale[i-1] + scale[i])/2)) / 100; //avg
			/*
			low_numerator = (input - boundary[i-1]);
			high_numerator = ((boundary[i] - input));
			difference = boundary[i] - boundary[i-1]; 
			result = ((low_numerator * scale[i-1]) + (high_numerator * scale[i])) / difference;
			*/
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
	//xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
	UARTprintf("@{\"pv\" : %d, \"inverter\" : %d, \"wind\" : %d, \"grid\" : %d, \"load\" : %d,}\n", 0, load_v_rms, load_i_rms, dist_v_rms, dist_i_rms);
	//UARTprintf("@{\"Load Volt\" : %d, \"Load Curr\" : %d, \"Dist Volt\" : %d, \"Dist Curr\" : %d,}\n", load_v_rms, load_i_rms, dist_v_rms, dist_i_rms);
	//UARTprintf("@{grid: \"load\" : %d}\n", (load_i_rms * load_v_rms)/1000);
	//UARTprintf("@{grid: \"load\" : %d}\n", load_v_rms);
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

