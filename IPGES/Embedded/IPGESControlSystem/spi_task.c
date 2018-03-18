//*****************************************************************************
//
// spi_task.c - SPI to talk to Kassandra Smith's SPI to Analog PV controller.
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
// 
//
//*****************************************************************************

static void SPITask(void *pvParameters)
{
		uint32_t pui32DataTx[NUM_SSI_DATA];
    uint32_t pui32DataRx[NUM_SSI_DATA];
    uint32_t ui32Index;
	   
		portTickType ui32WakeTime;
	
		xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
		UARTprintf("SPI Init\n");
		xSemaphoreGive(g_pUARTSemaphore);

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
	
	  pui32DataTx[0] = 33; //initial value
    //pui32DataTx[1] = 128;
    //pui32DataTx[2] = 0;
		
    // Loop forever.
    while(1)
    {  
			//pui32DataTx[0] = output;
			//output++;
			//output = output % 129;
		
			for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
			{
				//
				// Test Call
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
        vTaskDelayUntil(&ui32WakeTime, 100 / portTICK_RATE_MS); //Sleep Scheduler
    } //forever loop 
}



//*****************************************************************************
//
// Initializes the SPI Task.
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
									 SSI_MODE_MASTER, 976000, 16); // 1000000
	
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

