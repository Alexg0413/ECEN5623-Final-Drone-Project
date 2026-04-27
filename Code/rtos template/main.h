/*
 * main.h
 *
 *  Created on: Jun 14, 2018
 *      Author: Laptop
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>


// System clock rate, 120 MHz
#define SYSTEM_CLOCK    120000000U

// Set to 1 to enable all logging and UART print output, 0 to disable
#define DEBUG 1

typedef struct {
    uint8_t  data[8];
    uint32_t id;
    uint8_t  valid;
} CAN_Frame_t;

// Shared state arrays (defined in main.c, accessible to all tasks)
// state_vec[9]  — attitude, angular rates, z-position, z-velocity, z-acceleration (floats)
// input_vec[4]   — control inputs (floats)
// output_vec[4]  — motor commands (ints)
// switch_vec[3]  — aux switches (ints)
extern float state_vec[9];
extern float input_vec[4];
extern int   output_vec[4];
extern int   switch_vec[3];

extern volatile CAN_Frame_t can1;
extern volatile CAN_Frame_t can2;
extern volatile CAN_Frame_t can3;
extern volatile CAN_Frame_t can4;
extern volatile uint32_t g_ulCan1MsgCount;
extern volatile uint32_t g_ulCan1FreqHz;
extern volatile uint32_t g_ulCan1LastDelta_us;

float bytes_to_float(volatile uint8_t *bytes, uint8_t size, float scale_factor);


#endif /* MAIN_H_ */
