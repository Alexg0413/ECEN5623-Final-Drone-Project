/*
 * CAN.h
 *
 *  Created on: Mar 24, 2026
 *      Author: Ian Mcconachie
 */

#ifndef CAN_H_
#define CAN_H_

#include <stdint.h>

typedef struct {
    uint8_t  data[8];
    uint32_t id;
    uint8_t  valid;
} CAN_Frame_t;

extern volatile CAN_Frame_t can1;
extern volatile CAN_Frame_t can2;
extern volatile CAN_Frame_t can3;
extern volatile CAN_Frame_t can4;

extern volatile uint32_t g_ulCan1MsgCount;
extern volatile uint32_t g_ulCan1FreqHz;
extern volatile uint32_t g_ulCan1LastDelta_us;

// Initialize CAN1 peripheral, pins, interrupts, and message objects
void CAN1_init(uint32_t sysClock);

// Update CAN1 frequency profiler (no-op when DEBUG=0)
void CAN_update_freq(void);

float bytes_to_float(volatile uint8_t *bytes, uint8_t size, float scale_factor);

#endif /* CAN_H_ */
