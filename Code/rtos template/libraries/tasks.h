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

// task 1 - 200Hz
void State_input(void *pvParameters);

// task 2 - 50Hz
void Radio_Input(void *pvParameters);

// task 3 - 150Hz
void Motor_Output(void *pvParameters);

// task 4 - 100Hz
void Controller(void *pvParameters);

// logs per-event start/end/duration from circular buffer 
void vTimestampLoggingTask(void *pvParameters);

// logs running average duration per task 
void vWcetLoggingTask(void *pvParameters);

/////// helper functions

// setup the shared UART semaphore 
void vSetUARTSemaphore(SemaphoreHandle_t xSemaphore);

#endif /* TASKS_H_ */
