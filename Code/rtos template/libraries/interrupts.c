/*
 * interrupts.c
 *
 *  Created on: Mar 24, 2026
 *      Author: bprl-dev
 */

#include <libraries/interrupts.h>
#include <libraries/CAN.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/can.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"


// DWT register addresses (ARM Cortex-M4)
#define DWT_CYCCNT  (*((volatile uint32_t *)0xE0001004))
#define DWT_CTRL    (*((volatile uint32_t *)0xE0001000))
#define DEMCR       (*((volatile uint32_t *)0xE000EDFC))


// --------------------------------- 
// 600Hz = 1.6667 ms
// 120 MHz clock: 120,000,000 / 600 = 200,000 ticks per period
#define SEQUENCER_PERIOD_TICKS 200000 // 1.6667 ms period at 120 MHz

#define RX_BUFFER_SIZE 256

volatile uint8_t canBuffer[RX_BUFFER_SIZE];

// semaphore table (seqgen.c)
static SeqEntry_t xSeqTable[NUM_SERVICES] =
{
    {3, 0, NULL},  // Service_1 - 200 Hz
    {12, 0, NULL}, // Service_2 - 50 Hz 
    {4, 0, NULL},  // Service_3 - 150 Hz
    {6, 0, NULL},  // Service_4 - 100 Hz
    {300, 0, NULL}, //  log     - 5 Hz
};
// ---------------------------------

// configures TimerA0 for interrupts
void Timer0A_init(SemaphoreHandle_t sems[], uint32_t num_sems)
{
    uint32_t i;
    for (i = 0; i < num_sems && i < NUM_SERVICES; i++)
    {
        xSeqTable[i].sem = sems[i];
    }

    // enable the timer peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) {} // wait for the timer to be ready

    // configure periodic timer 
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Set the period
    TimerLoadSet(TIMER0_BASE, TIMER_A, SEQUENCER_PERIOD_TICKS - 1);

    // enable the timer interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // set the interrupt priority to 7 (lowest priority)
    IntPrioritySet(INT_TIMER0A, 0xE0); 
    
    // enable the interrupt and starts timer
    IntEnable(INT_TIMER0A);
    TimerEnable(TIMER0_BASE, TIMER_A);

}

// ISR, called by hardware every X ms based on the timer configuration
// uses semaphore table to release tasks ( this is the sequencer )
void Timer0AIntHandler(void)
{
    // clear the interrupt flag 
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // send timestamp to Q1 timer task queue
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t i;
    for (i = 0; i < NUM_SERVICES; i++)
    {
        xSeqTable[i].count++;
        if (xSeqTable[i].count >= xSeqTable[i].period)
        {
            // reset count 
            xSeqTable[i].count = 0;

            // send signal to corresponding task 
            if (xSeqTable[i].sem != NULL)
            {
                xSemaphoreGiveFromISR(xSeqTable[i].sem, &xHigherPriorityTaskWoken);
            }
        }
    }

    // yields to the higher priority task if it was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void CAN1IntHandler(void)
{
    uint32_t status;

    status = CANIntStatus(CAN1_BASE, CAN_INT_STS_CAUSE);

    // status interrupt (CAN error, bus-off, etc.) — clear and return
    if(status == 0x8000U)
    {
        CANStatusGet(CAN1_BASE, CAN_STS_CONTROL);
        return;
    }

    if(status == 0)
        return;

    tCANMsgObject msg;
    uint8_t data[8];
    uint8_t i;

    msg.pui8MsgData = data;

    CANMessageGet(CAN1_BASE, status, &msg, true);

    switch(msg.ui32MsgID)
    {
        case 0x01:
            for (i = 0; i < 8; i++) can1.data[i] = data[i];
            can1.id = 0x01;
            can1.valid = 1;
            break;
            
            case 0x02:
            for (i = 0; i < 8; i++) can2.data[i] = data[i];
            can2.id = 0x02;
            can2.valid = 1;
            CAN_update_freq(); // used to measure CAN receive frequency
            break;

        case 0x03:
            for (i = 0; i < 8; i++) can3.data[i] = data[i];
            can3.id = 0x03;
            can3.valid = 1;
            break;

        case 0x04:
            for (i = 0; i < 8; i++) can4.data[i] = data[i];
            can4.id = 0x04;
            can4.valid = 1;
            break;
    }

    CANIntClear(CAN1_BASE, status);
}


void DWT_init(void)
{
    
    DEMCR       |= (1UL << 24);  // enable trace
    DWT_CYCCNT   = 0;            // reset counter
    DWT_CTRL    |= (1UL << 0);   // start counter
}

// used for 0.1 usec resolution timing with DWT cycle counter
uint32_t getTime_100ns(void)
{
    return DWT_CYCCNT / 12U; 
}

// used for 1 usec resolution timing with DWT cycle counter
uint32_t getTime_us(void)
{
    return DWT_CYCCNT / 120U; 
}

