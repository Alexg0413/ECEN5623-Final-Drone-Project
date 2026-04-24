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
    // UART2 on PA6/PA7,  PA0/PA1 are reserved for CAN0 (JP4,5 set to CAN)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART2)){}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){}

    // Configure GPIO Pins for UART2 (PA6=RX, PA7=TX)
    GPIOPinConfigure(GPIO_PA6_U2RX);
    GPIOPinConfigure(GPIO_PA7_U2TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    // Initialize the UART for console I/O.
    UARTConfigSetExpClk(UART2_BASE, g_ui32SysClock, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTStdioConfig(2, 115200, g_ui32SysClock);
    UARTEnable(UART2_BASE);

    // ---------------------------
    // Enable CAN0 
    // ---------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0));

    // Configure GPIO Pins for CAN0 (PA0=RX, PA1=TX)
    GPIOPinConfigure(GPIO_PA0_CAN0RX);
    GPIOPinConfigure(GPIO_PA1_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // init CAN and set bit rate to 1 Mbps
    CANInit(CAN0_BASE);
    CANBitRateSet(CAN0_BASE, g_ui32SysClock, 1000000); // 1 Mbps
    CANEnable(CAN0_BASE);

    // Send SET_DATA

    CANIntRegister(CAN0_BASE, CAN0_Handler);
    IntEnable(INT_CAN0);
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    // Send SET_DATA command
    tCANMsgObject txMsg;
    uint8_t txData[8];

    txMsg.ui32MsgID = 0x000;   // host → IMX (verify this)
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

    xTaskCreate(State_input,  "S1", configMINIMAL_STACK_SIZE, (void*)xSems[0], configMAX_PRIORITIES-1, NULL); // S1: 200 Hz
    xTaskCreate(Radio_Input,  "S2", configMINIMAL_STACK_SIZE, (void*)xSems[1], configMAX_PRIORITIES-2, NULL); // S2: 50 Hz
    xTaskCreate(Motor_Output, "S3", configMINIMAL_STACK_SIZE, (void*)xSems[2], configMAX_PRIORITIES-3, NULL); // S3: 150 Hz
    xTaskCreate(Controller,   "S4", configMINIMAL_STACK_SIZE, (void*)xSems[3], configMAX_PRIORITIES-2, NULL); // S4: 100 Hz

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