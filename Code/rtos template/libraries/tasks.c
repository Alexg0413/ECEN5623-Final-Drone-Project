/*
 * tasks.c
 *
 *  Created on: Mar 24, 2026
 *      Author: bprl-dev
 */

#include <libraries/interrupts.h>
#include <libraries/tasks.h>
#include <libraries/PWM.h>
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

// handle CAN bus input (interrupt)
// converts data from int to float 
// integrate acceleration to get velocity / position
// do transform to convert from body frame to world frame (position) 
// stores in shared state array
void State_input(void *pvParameters)
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

// handles radio PWM inputs
// converts control target data from int to float and stores in shared control input array
// converts aux switch data from PWM duty cycle to 1 or 0 (int) and stores in shared switch input array
void Radio_Input(void *pvParameters)
{
    SemaphoreHandle_t semaphore = (SemaphoreHandle_t)pvParameters;
    uint32_t ulStart;
    uint32_t ulEnd;
    uint32_t ulThread_id = 2;
    uint32_t pw[6];
    int i;

#if DEBUG
    static uint32_t lastPrint = 0;  // add this
#endif

    while(1)
    {
        xSemaphoreTake(semaphore, portMAX_DELAY);
        ulStart = getTime_100ns();

        taskENTER_CRITICAL();
        for (i = 0; i < 6; i++)
            pw[i] = pulse_width[i];
        taskEXIT_CRITICAL();

        float roll   = (pw[0] - 1500.0f) / 500.0f;
        float pitch  = -1*(pw[1] - 1500.0f) / 500.0f;
        float yaw    = (pw[3] - 1500.0f) / 500.0f;
        float thrust = (2000.0f-pw[2]) / 1000.0f;

        // float fcut = 10.0f
        // delta_time = 1.0f/150.0f; // assuming 150Hz update rate 

        // float RC = 1.0f / (2.0f * 3.14159265f * fcut);
        // roll += (delta_time / (RC + delta_time)) * (roll - last_roll);



        if (roll > 1.0f) roll = 1.0f;
        if (roll < -1.0f) roll = -1.0f;

        if (pitch > 1.0f) pitch = 1.0f;
        if (pitch < -1.0f) pitch = -1.0f;

        if (yaw > 1.0f) yaw = 1.0f;
        if (yaw < -1.0f) yaw = -1.0f;

        if (thrust > 1.0f) thrust = 1.0f;
        if (thrust < 0.0f) thrust = 0.0f;

        taskENTER_CRITICAL();
        input_vec[0] = thrust;
        input_vec[1] = roll;
        input_vec[2] = pitch;
        input_vec[3] = yaw;
        switch_vec[1] = (pw[5] > 1500);
        switch_vec[0] = (pw[4] > 1500);
        taskEXIT_CRITICAL();

#if DEBUG
        uint32_t now = xTaskGetTickCount();
        if ((now - lastPrint) > pdMS_TO_TICKS(200))
        {
            int roll_i   = (int)(roll * 1000.0f);
            int pitch_i  = (int)(pitch * 1000.0f);
            int yaw_i    = (int)(yaw * 1000.0f);
            int thrust_i = (int)(thrust * 1000.0f);

            xSemaphoreTake(xPrintSem, portMAX_DELAY);
            UARTprintf("Norm: R:%d P:%d Y:%d T:%d | SW:%d %d\r\n",
                    roll_i, pitch_i, yaw_i, thrust_i,
                    switch_vec[0], switch_vec[1]);
            xSemaphoreGive(xPrintSem);

            lastPrint = now;
        }
#endif

        ulEnd = getTime_100ns();

#if DEBUG

        vLogTiming(ulThread_id, ulStart, ulEnd);
#endif
    }
}
/*
// read motor PWM (ints) from shared control output array
// write the PWM duty cycle to the PWM generator registers to command the motors
void Motor_Output(void *pvParameters)
{
    SemaphoreHandle_t semaphore = (SemaphoreHandle_t)pvParameters;
    uint32_t ulStart;
    uint32_t ulEnd;
    uint32_t ulThread_id = 3;
    int motor_cmd[4];
    int arm;
    int i;

    while(1)
    {
        xSemaphoreTake(semaphore, portMAX_DELAY);
        ulStart = getTime_100ns();

        taskENTER_CRITICAL();
        for (i = 0; i < 4; i++)
            motor_cmd[i] = output_vec[i];
        arm = switch_vec[0];
        taskEXIT_CRITICAL();

        if (!arm)
        {
            for (i = 0; i < 4; i++)
                motor_cmd[i] = 1000;
        }

        for (i = 0; i < 4; i++)
        {
            if (motor_cmd[i] < 1000) motor_cmd[i] = 1000;
            if (motor_cmd[i] > 2000) motor_cmd[i] = 2000;
        }

        Motor_Update(motor_cmd);

        ulEnd = getTime_100ns();
#if DEBUG
        vLogTiming(ulThread_id, ulStart, ulEnd);
#endif
    }
}
*/
/*
void Motor_Output(void *pvParameters)
{
    SemaphoreHandle_t semaphore = (SemaphoreHandle_t)pvParameters;
    int motor_cmd[4] = {1300, 1300, 1300, 1300};

#if DEBUG
    UARTprintf("Motor_Output task started\r\n");
    static uint32_t lastPrint = 0;
#endif

    while(1)
    {
        xSemaphoreTake(semaphore, portMAX_DELAY);

        Motor_Update(motor_cmd);

#if DEBUG
        uint32_t now = xTaskGetTickCount();

        if ((now - lastPrint) > pdMS_TO_TICKS(200))
        {
            UARTprintf("Motor PWM: %d %d %d %d\r\n",
                       motor_cmd[0],
                       motor_cmd[1],
                       motor_cmd[2],
                       motor_cmd[3]);

            lastPrint = now;
        }
#endif
    }
}
*/


void Motor_Output(void *pvParameters)
{
    SemaphoreHandle_t semaphore = (SemaphoreHandle_t)pvParameters;
    int motor_cmd[4];
    int i;



    static int test_done = 0;

    while(1)
    {
        xSemaphoreTake(semaphore, portMAX_DELAY);

        if (!test_done)
        {
            int m;
            for (m = 0; m < 4; m++)
            {


                uint32_t start = xTaskGetTickCount();

                while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(3000))
                {
                    // all motors off
                    for (i = 0; i < 4; i++)
                        motor_cmd[i] = 1000;

                    // one motor on
                    motor_cmd[m] = 1300;

                    Motor_Update(motor_cmd);
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                // pause between motors
                for (i = 0; i < 4; i++)
                    motor_cmd[i] = 1000;

                Motor_Update(motor_cmd);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }


            test_done = 1;
        }


        for (i = 0; i < 4; i++)
            motor_cmd[i] = 1000;

        Motor_Update(motor_cmd);
    }
}
// reads shared state and control input arrays
// computes control outputs and mixes motor commands to get PWM values
// reads AUX switches and determines flight mode
// writes appropriate values to shared control output array
void Controller(void *pvParameters)
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



