/*
 * main.h
 *
 *  Created on: Jun 14, 2018
 *      Author: Laptop
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include <stdbool.h>

// System clock rate, 120 MHz
#define SYSTEM_CLOCK    120000000U

// Set to 1 to enable all logging and UART print output, 0 to disable
#define DEBUG 1


// state_vec[9]  — attitude, angular rates, z-position, z-velocity, z-acceleration
// input_vec[4]  — [thrust, roll_target, pitch_target, yaw_rate_target]
// output_vec[4] — [motor0, motor1, motor2, motor3]
// switch_vec[3] — [Arm, Aux1, Aux2]

extern float state_vec[9];
extern float input_vec[4];
extern int   output_vec[4];
extern int   switch_vec[2];



// Raw pulse widths from receiver (microseconds)
extern volatile uint32_t pulse_width[6];

// Internal capture timing (used in ISR)
extern volatile uint32_t rise_time[6];
extern volatile bool rising_edge[6];

// PWM initialization
void PWM_Input_Init(void);

// Timer ISR handlers (must match vector table)
void Timer1AIntHandler(void);
void Timer1BIntHandler(void);
void Timer4AIntHandler(void);
void Timer4BIntHandler(void);
void Timer3AIntHandler(void);
void Timer3BIntHandler(void);

#endif /* MAIN_H_ */