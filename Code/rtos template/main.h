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
#include <libraries/CAN.h>

// System clock rate, 120 MHz
#define SYSTEM_CLOCK    120000000U

// Set to 1 to enable all logging and UART print output, 0 to disable
#define DEBUG 1

extern uint32_t g_ui32SysClock;

// state_vec[9]  — attitude, angular rates, z-position, z-velocity, z-acceleration
// input_vec[4]  — [thrust, roll_target, pitch_target, yaw_rate_target]
// output_vec[4] — [motor0, motor1, motor2, motor3]
// switch_vec[3] — [Arm, Aux1, Aux2]

// Shared state arrays (defined in main.c, accessible to all tasks)
extern float state_vec[9];
extern float input_vec[4];
extern int   output_vec[4];
extern int   switch_vec[2];





#endif /* MAIN_H_ */
