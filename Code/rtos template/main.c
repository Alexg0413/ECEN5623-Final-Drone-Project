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

volatile CAN_Frame_t can1;
volatile CAN_Frame_t can2;
volatile CAN_Frame_t can3;
volatile CAN_Frame_t can4;

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


    
    tCANMsgObject rxMsg1;
    uint8_t rxData1[8];
    rxMsg1.ui32MsgID = 0x01;
    rxMsg1.ui32MsgIDMask = 0x7FF;   // exact 11-bit match
    rxMsg1.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    rxMsg1.ui32MsgLen = 8;
    rxMsg1.pui8MsgData = rxData1;
    CANMessageSet(CAN1_BASE, 1, &rxMsg1, MSG_OBJ_TYPE_RX);

    tCANMsgObject rxMsg2;
    uint8_t rxData2[8];
    rxMsg2.ui32MsgID = 0x02;
    rxMsg2.ui32MsgIDMask = 0x7FF;   // exact 11-bit match
    rxMsg2.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    rxMsg2.ui32MsgLen = 8;
    rxMsg2.pui8MsgData = rxData2;
    CANMessageSet(CAN1_BASE, 2, &rxMsg2, MSG_OBJ_TYPE_RX);

    tCANMsgObject rxMsg3;
    uint8_t rxData3[8];
    rxMsg3.ui32MsgID = 0x03;
    rxMsg3.ui32MsgIDMask = 0x7FF;   // exact 11-bit match
    rxMsg3.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    rxMsg3.ui32MsgLen = 8;
    rxMsg3.pui8MsgData = rxData3;
    CANMessageSet(CAN1_BASE, 3, &rxMsg3, MSG_OBJ_TYPE_RX);

    tCANMsgObject rxMsg4;
    uint8_t rxData4[8];
    rxMsg4.ui32MsgID = 0x04;
    rxMsg4.ui32MsgIDMask = 0x7FF;   // exact 11-bit match
    rxMsg4.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    rxMsg4.ui32MsgLen = 8;
    rxMsg4.pui8MsgData = rxData4;
    CANMessageSet(CAN1_BASE, 4, &rxMsg4, MSG_OBJ_TYPE_RX);


    ///////////////////////////////////// semaphores
    // create semaphores for synchronizing tasks
    SemaphoreHandle_t xSems[NUM_SERVICES];
    int j;
    for (j = 0; j < NUM_SERVICES; j++)
        xSems[j] = xSemaphoreCreateBinary();

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
    int i;
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
    uint32_t status;

    status = CANIntStatus(CAN1_BASE, CAN_INT_STS_CAUSE);

    if(status == 0)
    {
        CANStatusGet(CAN1_BASE, CAN_STS_CONTROL);
        return;
    }

    tCANMsgObject msg;
    uint8_t data[8];

    msg.pui8MsgData = data;

    CANMessageGet(CAN1_BASE, status, &msg, true);

    switch(msg.ui32MsgID)
    {
        case 0x01:
            memcpy((void*)can1.data, data, 8);
            can1.id = 0x01;
            can1.valid = 1;
            break;

        case 0x02:
            memcpy((void*)can2.data, data, 8);
            can2.id = 0x02;
            can2.valid = 1;
            break;

        case 0x03:
            memcpy((void*)can3.data, data, 8);
            can3.id = 0x03;
            can3.valid = 1;
            break;

        case 0x04:
            memcpy((void*)can4.data, data, 8);
            can4.id = 0x04;
            can4.valid = 1;
            break;
    }

    CANIntClear(CAN1_BASE, status);
}


void State_input(void)
{
    // state_vec[9]  — attitude, angular rates, z-position, z-velocity, z-acceleration (floats)
    float euler1 = bytes_to_float(can1.data[0], 2, 10000)
    float euler2 = bytes_to_float(can1.data[2], 2, 10000)
    float euler3 = bytes_to_float(can1.data[4], 2, 10000)
    float p = bytes_to_float(can2.data[0], 2, 1000);
    float q = bytes_to_float(can3.data[0], 2, 1000);
    float r = bytes_to_float(can4.data[0], 2, 1000);
    float u_dot = bytes_to_float(can2.data[2], 2, 100);
    float v_dot = bytes_to_float(can3.data[2], 2, 100);
    float w_dot = bytes_to_float(can4.data[2], 2, 100);

    UARTprintf("Euler Angles: %f,%f,%f\r\n", euler1, euler2, euler3);
    UARTprintf("Angular Rates: %f,%f,%f\r\n", p, q, r);
    UARTprintf("Velocities: %f,%f,%f\r\n", u_dot, v_dot, w_dot);

    state[0] = euler1;
    state[1] = euler2;
    state[2] = euler3;
    state[3] = p;
    state[4] = q;
    state[5] = r;
    state[6] = u_dot;
    state[7] = v_dot;
    state[8] = w_dot;
}


// Help function
float bytes_to_float(uint8_t *bytes, uint8_t size, float scale_factor) {
    int16_t raw_val = 0;
    memcpy(bytes, raw_val, size);
    float f = raw_val / scale_factor;
    return f;
}