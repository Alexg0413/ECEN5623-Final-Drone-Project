/* FreeRTOS 8.2 Tiva Demo
 *
 * main.c
 *
 * Andy Kobyljanec
 *
 * This is a simple demonstration project of FreeRTOS 8.2 on the Tiva Launchpad
 * EK-TM4C1294XL.  TivaWare driverlib sourcecode is included.
 * 
 * Modified By Ian Mcconachie 
 * adding code for EX5 based on Sam Siewert's, Sequencer Generic 
 * 
 */
// Standard includes
#include <libraries/interrupts.h>
#include <libraries/tasks.h>
#include <stdint.h>
#include <stdbool.h>

// TivaWare includes
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h" 
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// project includes
#include "main.h"

uint32_t g_ui32SysClock;

// Main function
int main(void)
{

    ///////////////////////////////////// 
    //initializes the system 
    /////////////////////////////////////

    ///////////////////////////////////// Clock 
    // Initialize system clock to 120 MHz
	g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
	                                             SYSCTL_OSC_MAIN |
	                                             SYSCTL_USE_PLL |
	                                             SYSCTL_CFG_VCO_480), 120000000);


    ///////////////////////////////////// UART printing  
    // enable the peripherals used for printing to the console
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)){} // wait for UART0 to be ready
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){}// wait for GPIOA to be ready
        
    // Configure GPIO Pins for UART0
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O.
    UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTStdioConfig(0, 115200, g_ui32SysClock);
    UARTEnable(UART0_BASE); // enable the UART0 module

    ///////////////////////////////////// semaphores
    // create semaphores for synchronizing tasks
    int i;
    SemaphoreHandle_t xSems[NUM_SERVICES];
    for (i = 0; i < NUM_SERVICES; i++)
        xSems[i] = xSemaphoreCreateBinary();

    // initialize timer interrupt 
    Timer0A_init(xSems, NUM_SERVICES);
 
    // passes the UART semaphore to the tasks module so tasks can use it for synchronized printing
    SemaphoreHandle_t xPrintSem = xSemaphoreCreateBinary();
    vSetUARTSemaphore(xPrintSem); 
    xSemaphoreGive(xPrintSem); // give the print semaphore 
    
    // initialize DWT cycle counter for accurate timing
    DWT_init(); 


    /////////////////////////////////////
	// here we can create tasks
    /////////////////////////////////////

    // RM policy: T1 > T2 = T4 = T6 > T3 = T5 > T7
    xTaskCreate(vTask1, "T1", configMINIMAL_STACK_SIZE, (void*)xSems[0], configMAX_PRIORITIES-1, NULL); // T1: 3 Hz - highest
    xTaskCreate(vTask2, "T2", configMINIMAL_STACK_SIZE, (void*)xSems[1], configMAX_PRIORITIES-2, NULL); // T2: 1 Hz - second
    xTaskCreate(vTask3, "T3", configMINIMAL_STACK_SIZE, (void*)xSems[2], configMAX_PRIORITIES-3, NULL); // T3: 0.5 Hz - third
    xTaskCreate(vTask4, "T4", configMINIMAL_STACK_SIZE, (void*)xSems[3], configMAX_PRIORITIES-2, NULL); // T4: 1 Hz - same as T2
    xTaskCreate(vTask5, "T5", configMINIMAL_STACK_SIZE, (void*)xSems[4], configMAX_PRIORITIES-3, NULL); // T5: 0.5 Hz - same as T3
    xTaskCreate(vTask6, "T6", configMINIMAL_STACK_SIZE, (void*)xSems[5], configMAX_PRIORITIES-2, NULL); // T6: 1 Hz - same as T2
    xTaskCreate(vTask7, "T7", configMINIMAL_STACK_SIZE, (void*)xSems[6], configMAX_PRIORITIES-4, NULL); // T7: 0.1 Hz - lowest

    // logging task - max priority, triggered after longest period completes (logs every 10 seconds)
    // switch to vTimestampLoggingTask to change logging from timestamp log to WCET log
    xTaskCreate(vWcetLoggingTask, "Log", configMINIMAL_STACK_SIZE, (void*)xSems[7], 1, NULL);

    // start by posting all semaphores (critical instant)
    for (i = 0; i < NUM_SERVICES-1; i++)
        xSemaphoreGive(xSems[i]);

    // start scheduler
    vTaskStartScheduler();

	while(1) {} //you should never get here
    return 0;
}

/*  ASSERT() Error function
 *
 *  failed ASSERTS() from driverlib/debug.h are executed in this function
 */
void __error__(char *pcFilename, uint32_t ui32Line)
{
    // Place a breakpoint here to capture errors until logging routine is finished
    while (1)
    {
    }
}
