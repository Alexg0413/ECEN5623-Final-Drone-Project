/*
 * tasks.h
 *
 *  Created on: Mar 24, 2026
 *      Author: bprl-dev
 */

#ifndef TASKS_H_
#define TASKS_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"


/////// tasks 

// task 1, place holder 
void vTask1(void *pvParameters);

// task 2, place holder 
void vTask2(void *pvParameters);

// task 3, place holder 
void vTask3(void *pvParameters);

// task 4, place holder 
void vTask4(void *pvParameters);

// task 5, place holder 
void vTask5(void *pvParameters);

// task 6, place holder 
void vTask6(void *pvParameters);

// task 7, place holder 
void vTask7(void *pvParameters);

// logs per-event start/end/duration from circular buffer (slow sequencer)
void vTimestampLoggingTask(void *pvParameters);

// logs running average duration per task (fast sequencer)
void vWcetLoggingTask(void *pvParameters);

/////// helper functions

// setup the shared UART semaphore 
void vSetUARTSemaphore(SemaphoreHandle_t xSemaphore);


#endif /* TASKS_H_ */
