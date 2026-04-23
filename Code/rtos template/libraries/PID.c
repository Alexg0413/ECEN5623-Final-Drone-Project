/*
 * PID.c
 *
 *  Created on: Apr 21, 2026
 *      Author: bprl-dev
 */

#include <libraries/PID.h>
#include <libraries/interrupts.h>
#include <string.h>


#define STALE_TIMEOUT  2000000U  // reset integrator after 0.2 second of inactivity (100ns resolution)

void pid_init(PID_t *pid, float kp, float ki, float kd, float imax, float fcut)
{
    memset(pid, 0, sizeof(*pid));
    pid->kp   = kp;
    pid->ki   = ki;
    pid->kd   = kd;
    pid->imax = imax;
    pid->fcut = fcut;
    // last_t = 0 signals "never called" to pid_update
}

void pid_reset(PID_t *pid)
{
    pid->integrator      = 0.0f;
    pid->last_derivative = 0.0f;
    pid->deriv_valid     = 0;
}

float pid_update(PID_t *pid, float error)
{
    uint32_t tnow = getTime_100ns();
    uint32_t dt   = tnow - pid->last_t;
    float    output = 0.0f;

    // First call, or stale: reset integrator and suppress D
    if (pid->last_t == 0 || dt > STALE_TIMEOUT) {
        dt = 0;
        pid_reset(pid);
    }
    pid->last_t = tnow;

    float delta_time = (float)dt * 1e-7f;   // 100ns ticks → seconds

    // P term
    output += error * pid->kp;

    // D term with discrete low-pass filter
    if (pid->kd != 0.0f && dt > 0) {
        float derivative;

        if (!pid->deriv_valid) {
            // Suppress derivative on the first sample after a reset to avoid
            // a large D spike from a sudden error change 
            derivative           = 0.0f;
            pid->last_derivative = 0.0f;
            pid->deriv_valid     = 1;
        } else {
            derivative = (error - pid->last_error) / delta_time;
        }

        // Low-pass filter
        float RC = 1.0f / (2.0f * 3.14159265f * pid->fcut);
        pid->last_derivative += (delta_time / (RC + delta_time)) *
                                (derivative - pid->last_derivative);

        output += pid->kd * pid->last_derivative;
    }

    pid->last_error = error;

    // I term with anti-windup clamp
    if (pid->ki != 0.0f && dt > 0) {
        pid->integrator += error * pid->ki * delta_time;
        // clamps integrator 
        if      (pid->integrator >  pid->imax) pid->integrator =  pid->imax;
        else if (pid->integrator < -pid->imax) pid->integrator = -pid->imax;
        
        output += pid->integrator;
    }

    return output;
}
