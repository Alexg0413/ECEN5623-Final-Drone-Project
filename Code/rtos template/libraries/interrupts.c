/*
 * interrupts.c
 *
 *  Created on: Mar 24, 2026
 *      Author: bprl-dev
 */

#include <libraries/interrupts.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

// used for 0.1 usec resolution timing with DWT cycle counter
#define CYCLES_PER_100NS  (12U)   // 120,000,000 / 10,000,000

// DWT register addresses (ARM Cortex-M4)
#define DWT_CYCCNT  (*((volatile uint32_t *)0xE0001004))
#define DWT_CTRL    (*((volatile uint32_t *)0xE0001000))
#define DEMCR       (*((volatile uint32_t *)0xE000EDFC))

//// --------------------------------- seqgen.c timing
//// 30Hz = 33.333333 ms
//// 120 MHz clock: 120,000,000 / 30 = 4,000,000 ticks per period
//#define SEQUENCER_PERIOD_TICKS 4000000 // 33.33 ms period at 120 MHz
//
//// semaphore table (seqgen.c)
//static SeqEntry_t xSeqTable[NUM_SERVICES] =
//{
//    {10, 0, NULL},  // Service_1 - 3 Hz  , every 10th Sequencer loop
//    {30, 0, NULL},  // Service_2 - 1 Hz  , every 30th Sequencer loop
//    {60, 0, NULL},  // Service_3 - 0.5 Hz, every 60th Sequencer loop
//    {30, 0, NULL},  // Service_4 - 1 Hz  , every 30th Sequencer loop
//    {60, 0, NULL},  // Service_5 - 0.5 Hz, every 60th Sequencer loop
//    {30, 0, NULL},  // Service_6 - 1 Hz  , every 30th Sequencer loop
//    {300, 0, NULL}, // Service_7 - 0.1 Hz, every 300th Sequencer loop
//    {300, 0, NULL}, // WCET log  - 0.1Hz , dump after longest period completes
//};
//// ---------------------------------

// // --------------------------------- seqgen2x.c timing
// // 60Hz = 16.6666667 ms
// // 120 MHz clock: 120,000,000 / 60 = 2,000,000ticks per period
// # define SEQUENCER_PERIOD_TICKS 2000000 // 16.66ms period
//
// // semaphore table (seqgen2x.c)
// static SeqEntry_t xSeqTable[NUM_SERVICES] =
// {
//     {2, 0, NULL},   // Service_1 - 30 Hz, every other Sequencer loop
//     {6, 0, NULL},   // Service_2 - 10 Hz, every 6th Sequencer loop
//     {12, 0, NULL},  // Service_3 - 5 Hz , every 12th Sequencer loop
//     {6, 0, NULL},   // Service_4 - 10 Hz, every 6th Sequencer loop
//     {12, 0, NULL},  // Service_5 - 5 Hz , every 12th Sequencer loop
//     {6, 0, NULL},   // Service_6 - 10 Hz, every 6th Sequencer loop
//     {60, 0, NULL},  // Service_7 - 1 Hz , every 60th Sequencer loop
//     {60, 0, NULL},  // WCET log  - 1 Hz , dump after longest period completes
// };
// // ---------------------------------

// --------------------------------- seqgen.c x100 timing
// 3000Hz = .33333333 ms
// 120 MHz clock: 120,000,000 / 3000 = 4,000ticks per period
#define SEQUENCER_PERIOD_TICKS 40000 // .3333 ms period at 120 MHz

// semaphore table (seqgen.c)
static SeqEntry_t xSeqTable[NUM_SERVICES] =
{
    {10, 0, NULL},  // Service_1 - 300 Hz , every 10th Sequencer loop
    {30, 0, NULL},  // Service_2 - 100 Hz , every 30th Sequencer loop
    {60, 0, NULL},  // Service_3 - 50 Hz  , every 60th Sequencer loop
    {30, 0, NULL},  // Service_4 - 100 Hz , every 30th Sequencer loop
    {60, 0, NULL},  // Service_5 - 50 Hz  , every 60th Sequencer loop
    {30, 0, NULL},  // Service_6 - 100 Hz , every 30th Sequencer loop
    {300, 0, NULL}, // Service_7 - 10 Hz  , every 300th Sequencer loop
    {300, 0, NULL}, // WCET log  - 10 Hz , dump after longest period completes
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

void DWT_init(void)
{
    
    DEMCR       |= (1UL << 24);  // enable trace
    DWT_CYCCNT   = 0;            // reset counter
    DWT_CTRL    |= (1UL << 0);   // start counter
}

uint32_t getTime_100ns(void)
{
    return DWT_CYCCNT / CYCLES_PER_100NS;
}
