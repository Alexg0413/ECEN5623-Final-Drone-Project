/*
 * PID.h
 *
 *  Created on: Apr 21, 2026
 *      Author: bprl-dev
 */

 // This PID controller is based on the PID implementation in Ardupilots PID library

#ifndef LIBRARIES_PID_H_
#define LIBRARIES_PID_H_

#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float imax;             // maximum value for the integrator term (anti-windup)
    float fcut;             // low-pass filter cutoff (Hz) for derivative
    float integrator;
    float last_error;
    float last_derivative;
    uint8_t deriv_valid;    // 0 until first post-reset derivative sample is taken
    uint32_t last_t;        // last call time in 100ns ticks 
} PID_t;

// initialize the PID controller with the given gains, integrator limit, and derivative filter cutoff
void  pid_init(PID_t *pid, float kp, float ki, float kd, float imax, float fcut);

// update the PID controller with the current error and return the control output
float pid_update(PID_t *pid, float error);

// reset the PID controller's internal state (integrator, last error, etc.)
void  pid_reset(PID_t *pid);


#endif /* LIBRARIES_PID_H_ */
