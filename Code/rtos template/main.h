/*
 * main.h
 *
 *  Created on: Jun 14, 2018
 *      Author: Laptop
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include <libraries/CAN.h>

// System clock rate, 120 MHz
#define SYSTEM_CLOCK    120000000U

// Set to 1 to enable all logging and UART print output, 0 to disable
#define DEBUG 1

// Shared state arrays (defined in main.c, accessible to all tasks)
extern float state_vec[9];
extern float input_vec[4];
extern int   output_vec[4];
extern int   switch_vec[3];


#endif /* MAIN_H_ */
