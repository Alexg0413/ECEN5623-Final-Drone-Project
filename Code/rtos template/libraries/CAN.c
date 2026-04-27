/*
 * CAN.c
 *
 *  Created on: Apr 22, 2026
 *      Author: Ian Mcconachie
 */

#include <libraries/CAN.h>
#include <libraries/interrupts.h>
#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"

// CAN1IntHandler is defined in interrupts.c
extern void CAN1IntHandler(void);

volatile CAN_Frame_t can1;
volatile CAN_Frame_t can2;
volatile CAN_Frame_t can3;
volatile CAN_Frame_t can4;

volatile uint32_t g_ulCan1MsgCount     = 0;
volatile uint32_t g_ulCan1FreqHz       = 0;
volatile uint32_t g_ulCan1LastDelta_us = 0;
static   uint32_t s_ulCan1LastTime     = 0;

// Updates msg count and computes receive frequency 
void CAN_update_freq(void)
{
#if DEBUG
    g_ulCan1MsgCount++;
    uint32_t now = getTime_100ns();
    if (s_ulCan1LastTime != 0)
    {
        uint32_t delta = now - s_ulCan1LastTime;
        if (delta > 0)
        {
            g_ulCan1FreqHz       = 10000000UL / delta;
            g_ulCan1LastDelta_us = delta / 10;
        }
    }
    s_ulCan1LastTime = now;
#endif
}

void CAN1_init(uint32_t sysClock)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN1));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    GPIOPinConfigure(GPIO_PB0_CAN1RX);
    GPIOPinConfigure(GPIO_PB1_CAN1TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //UARTprintf("Initializing CAN1\r\n");
    CANInit(CAN1_BASE);
    CANBitRateSet(CAN1_BASE, sysClock, 500000); // 500 kbps

    //UARTprintf("Registering CAN1 interrupt\r\n");
    CANIntRegister(CAN1_BASE, CAN1IntHandler);

    IntEnable(INT_CAN1);
    CANIntEnable(CAN1_BASE,
                 CAN_INT_MASTER |
                 CAN_INT_ERROR  |
                 CAN_INT_STATUS);

    CANEnable(CAN1_BASE);

    // Configure RX message objects 1-4 
    // common config
    tCANMsgObject rxMsg;
    uint8_t rxData[8];
    rxMsg.ui32MsgIDMask = 0x7FF;
    rxMsg.ui32Flags     = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    rxMsg.ui32MsgLen    = 8;
    rxMsg.pui8MsgData   = rxData;

    // CAN mailbox specific configuration
    // sets the ID to the CAN mailbox then the data object is changed and reused for the other mailboxes.
    rxMsg.ui32MsgID = 0x01; CANMessageSet(CAN1_BASE, 1, &rxMsg, MSG_OBJ_TYPE_RX);
    rxMsg.ui32MsgID = 0x02; CANMessageSet(CAN1_BASE, 2, &rxMsg, MSG_OBJ_TYPE_RX);
    rxMsg.ui32MsgID = 0x03; CANMessageSet(CAN1_BASE, 3, &rxMsg, MSG_OBJ_TYPE_RX);
    rxMsg.ui32MsgID = 0x04; CANMessageSet(CAN1_BASE, 4, &rxMsg, MSG_OBJ_TYPE_RX);
}

float bytes_to_float(volatile uint8_t *bytes, uint8_t size, float scale_factor)
{
    int16_t raw_val = 0;
    memcpy(&raw_val, (const void *)bytes, size);
    return raw_val / scale_factor;
}
