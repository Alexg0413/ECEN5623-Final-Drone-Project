/*
 * tasks.c
 *
 *  Created on: Mar 24, 2026
 *      Author: bprl-dev
 */

#include <libraries/interrupts.h>
#include <libraries/tasks.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/uartstdio.h"

// running average log 
#define MAX_TASKS 4

typedef struct {
    uint32_t count;       // number of samples collected
    uint32_t avg_ticks;   // running average in x100ns units
} TaskStats_t;

// per-task stats, indexed by task_id-1
static TaskStats_t xTask_stats[MAX_TASKS];

// circular buffer timestamp log 
#define LOG_SIZE 256

typedef struct {
    uint32_t task_id;
    uint32_t start_ticks;
    uint32_t end_ticks;
} WcetLog_t;

static WcetLog_t xWcet_log[LOG_SIZE];
static uint32_t ulLog_idx = 0;

// printing semaphore, used to prevent intermingled print outputs
static SemaphoreHandle_t xPrintSem = NULL;

// helper function for logging
static inline void vLogTiming(uint32_t thread_id, uint32_t start, uint32_t end)
{
    uint32_t dur = end - start;
    uint32_t idx = thread_id - 1;

    xWcet_log[ulLog_idx].task_id     = thread_id;
    xWcet_log[ulLog_idx].start_ticks = start;
    xWcet_log[ulLog_idx].end_ticks   = end;
    ulLog_idx = (ulLog_idx + 1) % LOG_SIZE;

    xTask_stats[idx].count++;
    xTask_stats[idx].avg_ticks = (uint32_t)((int32_t)xTask_stats[idx].avg_ticks +
        ((int32_t)dur - (int32_t)xTask_stats[idx].avg_ticks) / (int32_t)xTask_stats[idx].count);
}

// creates UART (printing) semaphore, not needed but helps prevent intermingled print outputs
void vSetUARTSemaphore(SemaphoreHandle_t xSemaphore)
{
    xPrintSem = xSemaphore;
}

void vTask1(void *pvParameters)
{
    SemaphoreHandle_t semaphore = (SemaphoreHandle_t)pvParameters;
    uint32_t ulStart = 0;
    uint32_t ulEnd = 0;
    uint32_t ulThread_id = 1;

    while(1)
    {
        xSemaphoreTake(semaphore, portMAX_DELAY);
        ulStart = getTime_100ns();

        // ---------- task work here ----------
        

        ulEnd = getTime_100ns();
#if DEBUG
        vLogTiming(ulThread_id, ulStart, ulEnd);
#endif
    }
}

void vTask2(void *pvParameters)
{
    SemaphoreHandle_t semaphore = (SemaphoreHandle_t)pvParameters;
    uint32_t ulStart;
    uint32_t ulEnd;
    uint32_t ulThread_id = 2;


    while(1)
    {
        xSemaphoreTake(semaphore, portMAX_DELAY);
        ulStart = getTime_100ns();

        // ---------- task work here ----------

        ulEnd = getTime_100ns();
#if DEBUG
        vLogTiming(ulThread_id, ulStart, ulEnd);
#endif
    }
}

void vTask3(void *pvParameters)
{
    SemaphoreHandle_t semaphore = (SemaphoreHandle_t)pvParameters;
    uint32_t ulStart;
    uint32_t ulEnd;
    uint32_t ulThread_id = 3;


    while(1)
    {
        xSemaphoreTake(semaphore, portMAX_DELAY);
        ulStart = getTime_100ns();

        // ---------- task work here ----------

        ulEnd = getTime_100ns();
#if DEBUG
        vLogTiming(ulThread_id, ulStart, ulEnd);
#endif
    }
}

void vTask4(void *pvParameters)
{
    SemaphoreHandle_t semaphore = (SemaphoreHandle_t)pvParameters;
    uint32_t ulStart;
    uint32_t ulEnd;
    uint32_t ulThread_id = 4;

    while(1)
    {
        xSemaphoreTake(semaphore, portMAX_DELAY);
        ulStart = getTime_100ns();

        // ---------- task work here ----------

        ulEnd = getTime_100ns();
#if DEBUG
        vLogTiming(ulThread_id, ulStart, ulEnd);
#endif
    }
}

// Prints per-event start/end/duration from the circular buffer 
void vTimestampLoggingTask(void *pvParameters)
{
    SemaphoreHandle_t semaphore = (SemaphoreHandle_t)pvParameters;
    while(1)
    {
        xSemaphoreTake(semaphore, portMAX_DELAY);

#if DEBUG
        xSemaphoreTake(xPrintSem, portMAX_DELAY);
        uint32_t i;
        UARTprintf("---- Timestamp Log ----\n");
        for (i = 0; i < LOG_SIZE; i++)
        {
            if (xWcet_log[i].task_id != 0)
            {
                uint32_t duration = xWcet_log[i].end_ticks - xWcet_log[i].start_ticks;

                // convert x100ns ticks to ms (1ms = 10,000 ticks)
                uint32_t start_ms   = xWcet_log[i].start_ticks / 10000;
                uint32_t start_frac = xWcet_log[i].start_ticks % 10000;
                uint32_t end_ms     = xWcet_log[i].end_ticks   / 10000;
                uint32_t end_frac   = xWcet_log[i].end_ticks   % 10000;

                UARTprintf("Task %u: start=%u.%04ums, end=%u.%04ums, duration=%u (x100ns)\n",
                           xWcet_log[i].task_id,
                           start_ms, start_frac,
                           end_ms, end_frac,
                           duration);
            }
        }
        xSemaphoreGive(xPrintSem);
#endif
    }
}

// Prints the running average duration per task.
void vWcetLoggingTask(void *pvParameters)
{
    SemaphoreHandle_t semaphore = (SemaphoreHandle_t)pvParameters;
    while(1)
    {
        xSemaphoreTake(semaphore, portMAX_DELAY);

#if DEBUG
        xSemaphoreTake(xPrintSem, portMAX_DELAY);
        uint32_t i;
        UARTprintf("---- Avg Duration Log ----\n");
        for (i = 0; i < MAX_TASKS; i++)
        {
            if (xTask_stats[i].count > 0)
            {
                // logs the average duration in x100ns units and number of samples
                UARTprintf("Task %u: avg_duration=%u (x100ns), samples=%u\n",
                           i + 1,
                           xTask_stats[i].avg_ticks,
                           xTask_stats[i].count-1); // count starts at one so -1 for display pourposes
            }
        }
        xSemaphoreGive(xPrintSem);
#endif
    }
}



