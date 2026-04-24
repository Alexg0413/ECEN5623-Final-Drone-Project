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
#include <libraries/uart.h>
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
#include "inc/hw_ints.h"

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

void CAN1IntHandler(void);


// Main function
int main(void)
{
    uint32_t result = 0;

    result = SysCtlClockFreqSet(
        SYSCTL_XTAL_25MHZ |
        SYSCTL_OSC_MAIN |
        SYSCTL_USE_PLL |
        SYSCTL_CFG_VCO_480,
        120000000
    );

    if(result == 0)
    {
        UARTprintf("SysClock Error\r\n");
        while(1); // critical failure halt
    }

    g_ui32SysClock = result;

    UART0_init(g_ui32SysClock);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN1));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    // CAN1 Pin mux
    GPIOPinConfigure(GPIO_PB0_CAN1RX);
    GPIOPinConfigure(GPIO_PB1_CAN1TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // CAN1 Init
    UARTprintf("Initializing CAN1\r\n");
    CANInit(CAN1_BASE);
    CANBitRateSet(CAN1_BASE, g_ui32SysClock, 1000000);

    // Interrupt registration (check return)
    UARTprintf("Registering CAN1 interrupt\r\n");
    CANIntRegister(CAN1_BASE, CAN1IntHandler);
    
    // Interrupt enable
    IntEnable(INT_CAN1);
    CANIntEnable(CAN1_BASE,
                CAN_INT_MASTER |
                CAN_INT_ERROR |
                CAN_INT_STATUS);


    // Enable CAN
    CANEnable(CAN1_BASE);

    tCANMsgObject rxMsg;

    uint8_t rxData[8];

    rxMsg.ui32MsgID = 0x01;
    rxMsg.ui32MsgIDMask = 0x7FF;   // match full standard ID
    rxMsg.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    rxMsg.ui32MsgLen = 8;
    rxMsg.pui8MsgData = rxData;
    CANMessageSet(CAN1_BASE, 1, &rxMsg, MSG_OBJ_TYPE_RX);

    rxMsg.ui32MsgID = 0x02;
    CANMessageSet(CAN1_BASE, 1, &rxMsg, MSG_OBJ_TYPE_RX);

    rxMsg.ui32MsgID = 0x03;
    CANMessageSet(CAN1_BASE, 1, &rxMsg, MSG_OBJ_TYPE_RX);

    // // Send SET_DATA command
    // UARTprintf("Sending SET_DATA\r\n");
    // int i;
    // tCANMsgObject txMsg;
    // uint8_t txData[8];

    // txMsg.ui32MsgID = 0x000;   // host → IMX (verify this)
    // txMsg.ui32Flags = MSG_OBJ_NO_FLAGS;
    // txMsg.ui32MsgLen = 8;
    // txMsg.pui8MsgData = txData;

    // uint8_t packet[24];

    // // ---- Header ----
    // packet[0] = 0xFF;
    // packet[1] = 0x00;
    // packet[2] = DID_SET_DATA;   // DID_SET_DATA (LSB)
    // packet[3] = 0x00;

    // packet[4] = 16;     // payload size
    // packet[5] = 0x00;

    // packet[6] = 0x00;
    // packet[7] = 0x00;

    // // ---- Payload ----
    // uint32_t dataId = DID_INS_1;     // DID_INS_1 (VERIFY THIS)
    // uint32_t offset = 0;
    // uint32_t size   = 0;
    // uint32_t period = 100;   // 10 Hz

    // memcpy(&packet[8],  &dataId, 4);
    // memcpy(&packet[12], &offset, 4);
    // memcpy(&packet[16], &size,   4);
    // memcpy(&packet[20], &period, 4);

    // // ---- Send 3 CAN frames ----
    // for (i = 0; i < 24; i += 8)
    // {
    //     memcpy(txData, &packet[i], 8);
    //     CANMessageSet(CAN1_BASE, 1, &txMsg, MSG_OBJ_TYPE_TX);

    //     // wait for TX complete (important on Tiva)
    //     while (CANStatusGet(CAN1_BASE, CAN_STS_TXREQUEST));
    // }
    // UARTprintf("Sent SET_DATA\r\n");

    // ---------------------------
    // Interrupt Driven
    // ---------------------------
    UARTprintf("Enabling CAN0 Interrupt\r\n");
    IntEnable(INT_CAN1);
    CANIntEnable(CAN1_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);



    ///////////////////////////////////// semaphores
    // create semaphores for synchronizing tasks
    SemaphoreHandle_t xSems[NUM_SERVICES];
    int i;
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
    // xTaskCreate(vWcetLoggingTask, "Log", configMINIMAL_STACK_SIZE, (void*)xSems[7], 1, NULL); // 5Hz

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

void CAN1IntHandler(void)
{
    int i;
    uint32_t status = CANIntStatus(CAN1_BASE, CAN_INT_STS_CAUSE);
    if (status == 2)  // message object ID (your rx object)
    {
        tCANMsgObject rxMsg;
        uint8_t rxData[8];

        rxMsg.pui8MsgData = rxData;

        // Read message (this clears interrupt for that object)
        CANMessageGet(CAN1_BASE, 2, &rxMsg, true);

        UARTprintf("Printing received CAN Message:\r\n");
        // ---- Your processing ----
        // Example: print or buffer
        for (i = 0; i < rxMsg.ui32MsgLen; i++)
        {
            UARTprintf("%02X, " ,rxData[i]);
            // canBuffer[writeIdx++ % RX_BUFFER_SIZE] = rxData[i];
        }
        UARTprintf("\r\n");
    }

    // Clear interrupt
    CANIntClear(CAN1_BASE, status);
}