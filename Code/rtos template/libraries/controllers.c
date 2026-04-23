/*
 * controllers.c
 *
 *  Created on: Apr 22, 2026
 *      Author: Ian Mcconachie
 */

#include <libraries/controllers.h>
#include <libraries/PID.h>

// One PID instance per control axis
static PID_t roll_pid;
static PID_t pitch_pid;
static PID_t roll_rate_pid;
static PID_t pitch_rate_pid;
static PID_t yaw_rate_pid;
static PID_t climb_rate_pid;
static PID_t climb_accel_pid;

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

void attitude_controllers_update(float state_vec[], float input_vec[], int output_vec[])
{
    // Compute error for attitude outer control loop (roll, pitch) 
    // error = setpoint (pilot input) - measured state
    float roll_err  = input_vec[1] - state_vec[0];  
    float pitch_err = input_vec[2] - state_vec[1];  
    float yaw_rate_err  = input_vec[3] - state_vec[5]; 
    
    float roll_rate_trg   = pid_update(&roll_pid,   roll_err);
    float pitch_rate_trg  = pid_update(&pitch_pid,  pitch_err);
    
    float roll_rate_err   = roll_rate_trg - state_vec[3];  
    float pitch_rate_err  = pitch_rate_trg - state_vec[4];  
    
    float roll_rate_out   = pid_update(&roll_pid,   roll_err);
    float pitch_rate_out  = pid_update(&pitch_pid,  pitch_err);
    float yaw_rate_out  = pid_update(&yaw_rate_pid,    yaw_rate_err);

}


// TODO: add global variable for roll,pitch,yaw torque (output from attitude_controllers_update)

// TODO: create Z-axis controller (Not PID) to calculate thrust command from thrust input
//   expo curve controller for pilot input to thrust output (feedforward)

// TODO: add altitude hold controller (PID)

// TODO: Motor mixing: map roll, pitch, yaw PID outputs to PWM commands based on frame geometry
//   handle angle boost for thrust_out
//   add X-frame mixer: double check geometry( not quite a true X-frame 80deg between roll arms)
//      output_vec[0] = (int)(thrust_out + roll_out - pitch_out + yaw_out);  // FL
//      output_vec[1] = (int)(thrust_out - roll_out - pitch_out - yaw_out);  // FR
//      output_vec[2] = (int)(thrust_out + roll_out + pitch_out - yaw_out);  // RL
//      output_vec[3] = (int)(thrust_out - roll_out + pitch_out + yaw_out);  // RR