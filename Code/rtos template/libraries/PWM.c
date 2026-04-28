#include "PWM.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "FreeRTOS.h"
#include "main.h"


// ===== GLOBALS =====
uint32_t pwmPeriod;
uint32_t pwmClock;

volatile uint32_t rise_time[6];
volatile bool rising_edge[6] = {true,true,true,true,true,true};

#define PWM_FREQUENCY 150

volatile uint32_t pulse_width[6];

void Timer5AIntHandler(void)
{
    TimerIntClear(TIMER5_BASE, TIMER_CAPA_EVENT);// clear interrupt once triggered

    uint32_t time = TimerValueGet(TIMER5_BASE, TIMER_A);// to be used for calculations 

    if (rising_edge[0])// if it rises then we know its the start of the pwm 
    {
        rise_time[0] = time;
        TimerControlEvent(TIMER5_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
        rising_edge[0] = false;
    }
    else
    {
        uint32_t pw = (rise_time[0] - time) / 120;
        if (pw >= 800 && pw <= 2200)
            pulse_width[0] = pw;
        TimerControlEvent(TIMER5_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
        rising_edge[0] = true;
    }
}


void Timer5BIntHandler(void)
{
    TimerIntClear(TIMER5_BASE, TIMER_CAPB_EVENT);

    uint32_t time = TimerValueGet(TIMER5_BASE, TIMER_B);

    if (rising_edge[1])
    {
        rise_time[1] = time;
        TimerControlEvent(TIMER5_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);
        rising_edge[1] = false;
    }
    else
    {
        uint32_t pw = (rise_time[1] - time) / 120;
        if (pw >= 800 && pw <= 2200)
            pulse_width[1] = pw;
        TimerControlEvent(TIMER5_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);
        rising_edge[1] = true;
    }
}

// CH5 to Timer3A
void Timer3AIntHandler(void)
{
    TimerIntClear(TIMER3_BASE, TIMER_CAPA_EVENT);

    uint32_t time = TimerValueGet(TIMER3_BASE, TIMER_A);

    if (rising_edge[2])
    {
        rise_time[2] = time;
        TimerControlEvent(TIMER3_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
        rising_edge[2] = false;
    }
    else
    {
        uint32_t pw = (rise_time[2] - time) / 120;
        if (pw >= 800 && pw <= 2200)
            pulse_width[2] = pw;
        TimerControlEvent(TIMER3_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
        rising_edge[2] = true;
    }
}

// CH6  Timer3B
void Timer3BIntHandler(void)
{
    TimerIntClear(TIMER3_BASE, TIMER_CAPB_EVENT);

    uint32_t time = TimerValueGet(TIMER3_BASE, TIMER_B);

    if (rising_edge[3])
    {
        rise_time[3] = time;
        TimerControlEvent(TIMER3_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);
        rising_edge[3] = false;
    }
    else
    {
        uint32_t pw = (rise_time[3] - time) / 120;
        if (pw >= 800 && pw <= 2200)
            pulse_width[3] = pw;
        TimerControlEvent(TIMER3_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);
        rising_edge[3] = true;
    }
}

void Timer4AIntHandler(void)
{
    TimerIntClear(TIMER4_BASE, TIMER_CAPA_EVENT);

    uint32_t time = TimerValueGet(TIMER4_BASE, TIMER_A);

    if (rising_edge[4])
    {
        rise_time[4] = time;
        TimerControlEvent(TIMER4_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
        rising_edge[4] = false;
    }
    else
    {
        uint32_t pw = (rise_time[4] - time) / 120;
        if (pw >= 800 && pw <= 2200)
            pulse_width[4] = pw;
        TimerControlEvent(TIMER4_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
        rising_edge[4] = true;
    }
}

void Timer4BIntHandler(void)
{
    TimerIntClear(TIMER4_BASE, TIMER_CAPB_EVENT);

    uint32_t time = TimerValueGet(TIMER4_BASE, TIMER_B);

    if (rising_edge[5])
    {
        rise_time[5] = time;
        TimerControlEvent(TIMER4_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);
        rising_edge[5] = false;
    }
    else
    {
        uint32_t pw = (rise_time[5] - time) / 120;
        if (pw >= 800 && pw <= 2200)
            pulse_width[5] = pw;
        TimerControlEvent(TIMER4_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);
        rising_edge[5] = true;
    }
}

void PWM_Input_Init(void)
{

    // we need to turn on 6 timers that will read the PWM of 6 channels 

   // Turn on hardware blocks (ONLY what you actually use)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);


    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);// hardware is off if not on
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);  // for PA6, PA7
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);  // for PB2, PB3
   

    // Wait until ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER5));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER4));

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM));

    //make gpio pins to timer capture input 
    // Timer5
    GPIOPinConfigure(GPIO_PB2_T5CCP0);  // CH1
    GPIOPinConfigure(GPIO_PB3_T5CCP1);  // CH2
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3); 

    // Timer3
    GPIOPinConfigure(GPIO_PA6_T3CCP0);  // CH3
    GPIOPinConfigure(GPIO_PA7_T3CCP1);  // CH4
    GPIOPinTypeTimer(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    // Timer4: PM4=T4CCP0, PM5=T4CCP1
    GPIOPinConfigure(GPIO_PM4_T4CCP0);  // CH5
    GPIOPinConfigure(GPIO_PM5_T4CCP1);  // CH6
    GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_4 | GPIO_PIN_5);


    // congigure timers to be in capture mode check edges, measure time between edges 

    TimerConfigure(TIMER5_BASE, TIMER_CFG_SPLIT_PAIR |TIMER_CFG_A_CAP_TIME |TIMER_CFG_B_CAP_TIME);

    TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR |TIMER_CFG_A_CAP_TIME |TIMER_CFG_B_CAP_TIME);

    TimerConfigure(TIMER4_BASE,TIMER_CFG_SPLIT_PAIR |TIMER_CFG_A_CAP_TIME |TIMER_CFG_B_CAP_TIME);


    // Add after each TimerConfigure(), before TimerLoadSet():
    TimerPrescaleSet(TIMER5_BASE, TIMER_A, 119); // 120MHz/120 = 1MHz 
    TimerPrescaleSet(TIMER5_BASE, TIMER_B, 119);// timers run at 1MHz ticks now 1 tick from each timer is 1 microsecond, this makes the calculations easier later on 
    TimerPrescaleSet(TIMER3_BASE, TIMER_A, 119);
    TimerPrescaleSet(TIMER3_BASE, TIMER_B, 119);
    TimerPrescaleSet(TIMER4_BASE, TIMER_A, 119);
    TimerPrescaleSet(TIMER4_BASE, TIMER_B, 119);

    // Then 0xFFFF gives 65.5ms range  it is more than enough for 20ms PWM period 
    TimerLoadSet(TIMER5_BASE, TIMER_A, 0xFFFF);
    TimerLoadSet(TIMER5_BASE, TIMER_B, 0xFFFF);

    TimerLoadSet(TIMER3_BASE, TIMER_A, 0xFFFF);
    TimerLoadSet(TIMER3_BASE, TIMER_B, 0xFFFF);

    TimerLoadSet(TIMER4_BASE, TIMER_A, 0xFFFF);
    TimerLoadSet(TIMER4_BASE, TIMER_B, 0xFFFF);



    // edge detection we start on rising edge low to high  for all 6 ccps 
    TimerControlEvent(TIMER5_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerControlEvent(TIMER5_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);

    TimerControlEvent(TIMER3_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerControlEvent(TIMER3_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);

    TimerControlEvent(TIMER4_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerControlEvent(TIMER4_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);  


    // enable interrupts
    TimerIntEnable(TIMER5_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);
    TimerIntEnable(TIMER3_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);
    TimerIntEnable(TIMER4_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);

    //initialize interrupts 
    IntEnable(INT_TIMER5A);
    IntEnable(INT_TIMER5B);
    IntEnable(INT_TIMER3A);
    IntEnable(INT_TIMER3B);
    IntEnable(INT_TIMER4A);
    IntEnable(INT_TIMER4B);
    
    IntPrioritySet(INT_TIMER5A, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(INT_TIMER5B, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(INT_TIMER3A, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(INT_TIMER3B, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(INT_TIMER4A, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(INT_TIMER4B, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    
    IntMasterEnable();//enable global interrupt 


    // Start timers
    TimerEnable(TIMER5_BASE, TIMER_BOTH);
    TimerEnable(TIMER3_BASE, TIMER_BOTH);
    TimerEnable(TIMER4_BASE, TIMER_BOTH);
}

void PWM_Output_Init(void)
{
    // Enable PWM module + GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);// enable the PWM peripherals

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));

    // Set PWM clock = system clock / 64 (TM4C1294 requires PWMClockSet, not SysCtlPWMClockSet)
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_64);

    pwmClock = SYSTEM_CLOCK / 64; //use a slower clock cause the you need amounts of counts and you would need 
    //300000 to get what we need only have 65535

    // Compute period for 150 Hz
    pwmPeriod = pwmClock / PWM_FREQUENCY;

    //GPIO Congfiguration
    GPIOPinConfigure(GPIO_PG0_M0PWM4);
    GPIOPinConfigure(GPIO_PG1_M0PWM5);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIOPinConfigure(GPIO_PK4_M0PWM6);
    GPIOPinConfigure(GPIO_PK5_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Both generators control 2 outputs 
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, pwmPeriod);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, pwmPeriod);

    // Set them with initial PWM 
    uint32_t pulse_1000us = (pwmClock * 1000) / 1000000;

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pulse_1000us);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, pulse_1000us);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, pulse_1000us);// set all 4 with no throwttle setting 
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, pulse_1000us);

    // Enable outputs
    PWMOutputState(PWM0_BASE,PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT,true);

    // Start generators
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}

//how we will update the motor output 
void Motor_Update(int *cmd)
{
   
    uint32_t pulse;
    //cmd is in microseconds, need to convert to HZ, 
    pulse = (pwmClock * (uint32_t)cmd[0]) / 1000000;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, pulse);

    pulse = (pwmClock * (uint32_t)cmd[1]) / 1000000;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, pulse);

    pulse = (pwmClock * (uint32_t)cmd[2]) / 1000000;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, pulse);

    pulse = (pwmClock * (uint32_t)cmd[3]) / 1000000;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pulse);
}

