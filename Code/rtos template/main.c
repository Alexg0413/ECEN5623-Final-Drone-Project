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
#include "driverlib/can.h"
#include "driverlib/interrupt.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// project includes
#include "main.h"

uint32_t g_ui32SysClock;

#define DID_SET_DATA  3
#define DID_INS_1     4   // verify in your firmware
#define DID_IMU       58  // verify

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

    // ---------------------------
    // Enable CAN0 and GPIOB
    // ---------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Wait until ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    // Configure pins for CAN
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);

    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Init CAN Module
    CANInit(CAN0_BASE);

    // Set Bit Rate
    uint32_t sysClock = SysCtlClockGet();  // or SysCtlClockFreqSet for TM4C129
    uint32_t uiBitRate = 500000; // 500 kbps
    CANBitRateSet(CAN0_BASE, sysClock, uiBitRate); 

    // Enable Module
    CANEnable(CAN0_BASE);

    // Send SET_DATA command
    tCANMsgObject txMsg;
    uint8_t txData[8];

    txMsg.ui32MsgID = 0x100;   // host → IMX (verify this)
    txMsg.ui32Flags = MSG_OBJ_NO_FLAGS;
    txMsg.ui32MsgLen = 8;
    txMsg.pui8MsgData = txData;

    uint8_t packet[24];

    // ---- Header ----
    packet[0] = 0xFF;
    packet[1] = 0x00;
    packet[2] = DID_SET_DATA;   // DID_SET_DATA (LSB)
    packet[3] = 0x00;

    packet[4] = 16;     // payload size
    packet[5] = 0x00;

    packet[6] = 0x00;
    packet[7] = 0x00;

    // ---- Payload ----
    uint32_t dataId = DID_INS_1;     // DID_INS_1 (VERIFY THIS)
    uint32_t offset = 0;
    uint32_t size   = 0;
    uint32_t period = 100;   // 10 Hz

    memcpy(&packet[8],  &dataId, 4);
    memcpy(&packet[12], &offset, 4);
    memcpy(&packet[16], &size,   4);
    memcpy(&packet[20], &period, 4);

    // ---- Send 3 CAN frames ----
    for (int i = 0; i < 24; i += 8)
    {
        memcpy(txData, &packet[i], 8);
        CANMessageSet(CAN0_BASE, 1, &txMsg, MSG_OBJ_TYPE_TX);

        // wait for TX complete (important on Tiva)
        while (CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST));
    }

    // ---------------------------
    // Interrupt Driven
    // ---------------------------
    IntEnable(INT_CAN0);
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);



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

    // change vTaskX function to task functions once created 
    xTaskCreate(vTask1, "S1", configMINIMAL_STACK_SIZE, (void*)xSems[0], configMAX_PRIORITIES-1, NULL); // S1: 200 Hz - highest
    xTaskCreate(vTask2, "S2", configMINIMAL_STACK_SIZE, (void*)xSems[1], configMAX_PRIORITIES-2, NULL); // S2: 50 Hz - second
    xTaskCreate(vTask3, "S3", configMINIMAL_STACK_SIZE, (void*)xSems[2], configMAX_PRIORITIES-3, NULL); // S3: 150 Hz - third
    xTaskCreate(vTask4, "S4", configMINIMAL_STACK_SIZE, (void*)xSems[3], configMAX_PRIORITIES-2, NULL); // S4: 100 Hz - same as T2

    // logging task - min priority, 
    xTaskCreate(vWcetLoggingTask, "Log", configMINIMAL_STACK_SIZE, (void*)xSems[7], 1, NULL); // 5Hz

    // start by posting all semaphores 
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

#define RX_BUFFER_SIZE 256

volatile uint8_t canBuffer[RX_BUFFER_SIZE];

void CAN0IntHandler(void)
{
    uint32_t status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
    if (status == 2)  // message object ID (your rx object)
    {
        tCANMsgObject rxMsg;
        uint8_t rxData[8];

        rxMsg.pui8MsgData = rxData;

        // Read message (this clears interrupt for that object)
        CANMessageGet(CAN0_BASE, 2, &rxMsg, true);

        UARTprintf("Printing received CAN Message:\r\n");
        // ---- Your processing ----
        // Example: print or buffer
        for (int i = 0; i < rxMsg.ui32MsgLen; i++)
        {
            UARTprintf("%02X, " ,rxData[i]);
            // canBuffer[writeIdx++ % RX_BUFFER_SIZE] = rxData[i];
        }
        UARTprintf("\r\n");
    }

    // Clear interrupt
    CANIntClear(CAN0_BASE, status);
}