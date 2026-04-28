/*
 * interrupts.h
 *
 *  Created on: Mar 24, 2026
 *      Author: Ian Mcconachie
 */

#ifndef HW3_INTERRUPTS_H_
#define HW3_INTERRUPTS_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define NUM_SERVICES 5

typedef struct
{
    uint32_t          period;   // period in ISR ticks (multiples of base rate)
    uint32_t          count;    // running count, private to ISR
    SemaphoreHandle_t sem;      // semaphore to release
} SeqEntry_t;


// configures TimerA0 for interrupts
void Timer0A_init(SemaphoreHandle_t sems[], uint32_t num_sems);

void CAN1IntHandler(void);

// initialize DWT cycle counter for accurate timing
void DWT_init(void);  

// get current time in 100 ns units using DWT cycle counter
uint32_t getTime_100ns(void);  

#endif /* HW3_INTERRUPTS_H_ */
