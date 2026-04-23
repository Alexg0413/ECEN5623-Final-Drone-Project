/*
 * main.h
 *
 *  Created on: Jun 14, 2018
 *      Author: Laptop
 */

#ifndef MAIN_H_
#define MAIN_H_


// System clock rate, 120 MHz
#define SYSTEM_CLOCK    120000000U

// Set to 1 to enable all logging and UART print output, 0 to disable
#define DEBUG 1

// Shared state arrays (defined in main.c, accessible to all tasks)
// state_vec[9]  — attitude, angular rates, z-position, z-velocity, z-acceleration (floats)
// input_vec[4]   — control inputs (floats)
// output_vec[4]  — motor commands (ints)
// switch_vec[3]  — aux switches (ints)
extern float state_vec[9];
extern float input_vec[4];
extern int   output_vec[4];
extern int   switch_vec[3];


#endif /* MAIN_H_ */
