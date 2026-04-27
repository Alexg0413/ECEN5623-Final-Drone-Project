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
#include "driverlib/pwm.h"


// TivaWare includes
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// project includes
#include "main.h"

#include "driverlib/timer.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"

#include <libraries/PWM.h>




float state_vec[9] = {0};  //  [roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, z_pos, z_velo, z_accel]
float input_vec[4]  = {0};  // [thrust, roll_target, pitch_target, yaw_rate_target]
int   output_vec[4] = {0};  // [motor0, motor1, motor2, motor3]
int   switch_vec[2] = {0};  // [Arm, Aux1, Aux2]
uint32_t g_ui32SysClock;

// Main function
int main(void)
{

    //////////////////////////////////////////////////////////// initializes the system (init)

    ////////////////////////////////////// Clock 
    // Initialize system clock to 120 MHz
	g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
	                                             SYSCTL_OSC_MAIN |
	                                             SYSCTL_USE_PLL |
	                                             SYSCTL_CFG_VCO_480), 120000000);


    ////////////////////////////////////// UART printing
    UART0_init(g_ui32SysClock);

    ////////////////////////////////////// semaphores
    // create semaphores for synchronizing tasks
    int i;
    SemaphoreHandle_t xSems[NUM_SERVICES];
    for (i = 0; i < NUM_SERVICES; i++)
        xSems[i] = xSemaphoreCreateBinary();

    // initialize timer interrupt for sequencer
    Timer0A_init(xSems, NUM_SERVICES);



    PWM_Output_Init();//motor outouts will initialize motors to pwm 1000 microS which is minimum throttle

    for (i = 0; i < 4; i++)
    {
    output_vec[i] = 1000;// we initialize motors to a safe value, ESC expects a
    }
    //ESC uses PWM protocol and it must detect safe low throttle signal so it can arm which it requires for 2 seconds 
    SysCtlDelay(g_ui32SysClock / 3 * 5);

    //sets up gpio and timers put this after so we don't receive data just yet 
   // PWM_Input_Init(); // this is for radio 

    // passes the UART semaphore to the tasks module so tasks can use it for synchronized printing
    SemaphoreHandle_t xPrintSem = xSemaphoreCreateBinary();
    vSetUARTSemaphore(xPrintSem); 
    xSemaphoreGive(xPrintSem); // give the print semaphore 
    
    // initialize DWT cycle counter for accurate timing
    DWT_init(); 



	//////////////////////////////////////////////////////////// create tasks here
    
    // change vTaskX function to task functions once created 
    xTaskCreate(State_input, "S1", configMINIMAL_STACK_SIZE, (void*)xSems[0], configMAX_PRIORITIES-1, NULL); // S1: 200 Hz - highest
    xTaskCreate(Radio_Input, "S2", configMINIMAL_STACK_SIZE, (void*)xSems[1], configMAX_PRIORITIES-2, NULL); // S2: 50 Hz - second
    xTaskCreate(Motor_Output, "S3", configMINIMAL_STACK_SIZE, (void*)xSems[2], configMAX_PRIORITIES-3, NULL); // S3: 150 Hz - third
    xTaskCreate(Controller, "S4", configMINIMAL_STACK_SIZE, (void*)xSems[3], configMAX_PRIORITIES-2, NULL); // S4: 100 Hz - same as T2

    // logging task - min priority, 
    xTaskCreate(vWcetLoggingTask, "Log", configMINIMAL_STACK_SIZE, (void*)xSems[7], 1, NULL); // 5Hz

    // start by posting all semaphores 
    for (i = 0; i < NUM_SERVICES-1; i++)
        xSemaphoreGive(xSems[i]);


    //////////////////////////////////////////////////////////// start scheduler (loop)
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


