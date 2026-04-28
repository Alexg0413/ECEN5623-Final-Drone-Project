/*
 * controllers.c
 *
 *  Created on: Apr 22, 2026
 *      Author: Ian Mcconachie
 */

#include <libraries/controllers.h>
#include <libraries/PID.h>
#include "main.h"
#include <math.h>

float attitude_commands[3] = {0,0,0};  // [roll_torque, pitch_torque, yaw_torque]
float thrust_command       = 0.0f; // normalized throttle [0, 1]

// One PID instance per control axis
static PID_t roll_pid;
static PID_t pitch_pid;
static PID_t roll_rate_pid;
static PID_t pitch_rate_pid;
static PID_t yaw_rate_pid;
// static PID_t climb_rate_pid;
// static PID_t climb_accel_pid;

// state vector: [roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, z_pos, z_velo, z_accel]
// input vector: [thrust, roll_target, pitch_target, yaw_rate_target]

void attitude_controllers_init(void)
{
    pid_init(&roll_pid,   KP_ROLL,   KI_ROLL,   KD_ROLL,   IMAX_ROLL,   FCUT_HZ);
    pid_init(&pitch_pid,  KP_PITCH,  KI_PITCH,  KD_PITCH,  IMAX_PITCH,  FCUT_HZ);
    pid_init(&roll_rate_pid,   KP_ROLL_RATE,   KI_ROLL_RATE,   KD_ROLL_RATE,   IMAX_ROLL_RATE,   FCUT_HZ);
    pid_init(&pitch_rate_pid,  KP_PITCH_RATE,  KI_PITCH_RATE,  KD_PITCH_RATE,  IMAX_PITCH_RATE,  FCUT_HZ);
    pid_init(&yaw_rate_pid,    KP_YAW_RATE,    KI_YAW_RATE,    KD_YAW_RATE,    IMAX_YAW_RATE,    FCUT_HZ);
}

void attitude_controllers_update(float state_vec[], float input_vec[])
{
    // Compute error for attitude outer control loop (roll, pitch) 
    // error = setpoint (pilot input) - measured state
    float roll_err  = input_vec[1] - state_vec[0];  
    float pitch_err = input_vec[2] - state_vec[1];  
    float yaw_rate_err  = 5.0f * input_vec[3] - state_vec[5]; 
    
    float roll_rate_trg   = pid_update(&roll_pid,   roll_err);
    float pitch_rate_trg  = pid_update(&pitch_pid,  pitch_err);
    
    float roll_rate_err   = roll_rate_trg - state_vec[3];  
    float pitch_rate_err  = pitch_rate_trg - state_vec[4];  
    
    float roll_rate_out   = pid_update(&roll_rate_pid,   roll_rate_err);
    float pitch_rate_out  = pid_update(&pitch_rate_pid,  pitch_rate_err);
    float yaw_rate_out  = pid_update(&yaw_rate_pid,    yaw_rate_err);

    attitude_commands[0] = roll_rate_out;
    attitude_commands[1] = pitch_rate_out;
    attitude_commands[2] = yaw_rate_out;

}

void z_stabilize_controller_update(float state_vec[], float input_vec[])
{
    // input form pilot [0,1]
    float thr_in = input_vec[0];

    // adapted form Ardupilot throttle curve:
    //
    // expo curve for throttle input make 50% stick = hover throttle but still maps [0,1] input to [0,1] output
    // expo = -(thr_mid - 0.5) / 0.375, clamped to +-1
    const float expo = -(THR_MID - 0.5f) / 0.375f;

    float thr_expo = thr_in * (1.0f - expo) + expo * thr_in * thr_in * thr_in;

    // angle boost 
    // adjust the throttle command based on tilt angle to maintain vertical thrust. boost = 1/cos(tilt)
    float cos_tilt     = fminf(cosf(state_vec[0]), cosf(state_vec[1]));
    float boost_factor = 1.0f / cos_tilt;
    if (boost_factor > 2.0f) boost_factor = 1.75f;
    if (boost_factor < 1.0f) boost_factor = 1.0f;

    thrust_command = thr_expo * boost_factor;

    // Hard-clamp output to motor-safe range.
    if (thrust_command > 1.0f) thrust_command = 1.0f;
    if (thrust_command < 0.0f) thrust_command = 0.0f;
}

// TODO: add altitude hold controller (PID)

void motor_mixing_update(int armed, float state_vec[])
{
    /*   Roll  right   (+): left motors (FL, RL) up, right motors (FR, RR) down
    *    Pitch up      (+): rear motors (FL, FR) up, front motors (RL, RR) down
    *    Yaw   CW      (+): FL & RR up (CW props), FR & RL down (CCW props)
    *
    *    FL [2]        FR [0]
    *          \       /
    *           [body]
    *          /       \
    *    RL [1]        RR [4]
    */
    float deg_lim = 1.3f; // ~80 deg

    if (armed==0 || fabsf(state_vec[0]) > deg_lim || fabsf(state_vec[1]) > deg_lim) 
    {
        output_vec[0] = PWM_MIN_US;
        output_vec[1] = PWM_MIN_US;
        output_vec[2] = PWM_MIN_US;
        output_vec[3] = PWM_MIN_US;
        return;
    }

    // Scale thrust [0,1] -> [PWM_MIN_US, PWM_MAX_US]
    float thr   = PWM_MIN_US + thrust_command * (float)(PWM_MAX_US - PWM_MIN_US);

    // Scale attitude commands +-1 -> +- MIXER_ATT_SCALE PWM counts
    float roll  = attitude_commands[0] * MIXER_ATT_SCALE;
    float pitch = attitude_commands[1] * MIXER_ATT_SCALE;
    float yaw   = attitude_commands[2] * MIXER_ATT_SCALE;

    // X-frame mixing
    float fl = thr + roll + pitch + yaw;   // front-left
    float fr = thr - roll + pitch - yaw;   // front-right
    float rl = thr + roll - pitch - yaw;   // rear-left
    float rr = thr - roll - pitch + yaw;   // rear-right

    // Clamp each motor to the valid PWM range
    if (fl < PWM_MIN_US) fl = PWM_MIN_US;  if (fl > PWM_MAX_US) fl = PWM_MAX_US;
    if (fr < PWM_MIN_US) fr = PWM_MIN_US;  if (fr > PWM_MAX_US) fr = PWM_MAX_US;
    if (rl < PWM_MIN_US) rl = PWM_MIN_US;  if (rl > PWM_MAX_US) rl = PWM_MAX_US;
    if (rr < PWM_MIN_US) rr = PWM_MIN_US;  if (rr > PWM_MAX_US) rr = PWM_MAX_US;

    output_vec[0] = (int)fr;
    output_vec[1] = (int)rl;
    output_vec[2] = (int)fl;
    output_vec[3] = (int)rr;
}

