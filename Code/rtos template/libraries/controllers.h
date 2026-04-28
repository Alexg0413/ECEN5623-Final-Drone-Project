/*
 * controllers.h
 *
 *  Created on: Apr 22, 2026
 *      Author: Ian Mcconachie
 */

#ifndef LIBRARIES_CONTROLLERS_H_
#define LIBRARIES_CONTROLLERS_H_

///////////////////////////////////// 
// PID gains 
//////////////////////////////////////
#define KP_ROLL     4.5f
#define KI_ROLL     0.0f
#define KD_ROLL     0.0f
#define IMAX_ROLL   1.0f

#define KP_PITCH    4.5f
#define KI_PITCH    0.0f
#define KD_PITCH    0.0f
#define IMAX_PITCH  1.0f

#define KP_ROLL_RATE     0.11f
#define KI_ROLL_RATE     0.09f
#define KD_ROLL_RATE     0.003f
#define IMAX_ROLL_RATE   0.5f

#define KP_PITCH_RATE    0.11f
#define KI_PITCH_RATE    0.09f
#define KD_PITCH_RATE    0.003f
#define IMAX_PITCH_RATE  0.5f

#define KP_YAW_RATE     0.1f
#define KI_YAW_RATE     0.05f
#define KD_YAW_RATE     0.0f
#define IMAX_YAW_RATE   0.5f

///////// used for z-pos hold controller 
// #define KP_CLIMB_RATE   0.0f
// #define KI_CLIMB_RATE   0.0f
// #define KD_CLIMB_RATE   0.0f
// #define IMAX_CLIMB_RATE 1.0f

// #define KP_CLIMB_ACCEL   0.0f
// #define KI_CLIMB_ACCEL   0.0f
// #define KD_CLIMB_ACCEL   0.0f
// #define IMAX_CLIMB_ACCEL 1.0f

// Derivative low-pass cutoff frequency (Hz) applied to all axes
#define FCUT_HZ     30.0f

/////////////////////////////////////
// Motor PWM range (microseconds — standard hobby ESC protocol)
/////////////////////////////////////
#define PWM_MIN_US       1080    // pulse width for motor off / min throttle
#define PWM_MAX_US       2000    // pulse width for full throttle

// Max PWM authority granted to each attitude axis.
// Attitude commands are normalized +-1; this scales them to a PWM delta.
// 250 -> full authority on one axis swings 25% of the PWM range.
#define MIXER_ATT_SCALE  300.0f
#define MIXER_YAW_SCALE 150.0f // yaw typically needs less authority than roll/pitch

#define YAW_GAIN_MULT 18.0f

/////////////////////////////////////
// Throttle parameters
/////////////////////////////////////

// Learned hover throttle fraction 
//Tune to match the fraction of full throttle that holds the vehicle level at hover.
#define THR_MID          0.45f // set between 0 and 0.5, typically around 0.3-0.4


// Initialize all PID controllers with above gains
void attitude_controllers_init(void);

// Read state_vec / input_vec, run all four PIDs, write results to output_vec
void attitude_controllers_update(float state_vec[], float input_vec[]);

void z_stabilize_controller_update(float state_vec[], float input_vec[]);

// Mix thrust + attitude commands into per-motor PWM values.
// Reads thrust_command, attitude_commands[], output_vec[], and switch_vec[] globals.
// Motors are forced to PWM_MIN_US when switch_vec[0] (arm channel) == 0.
void motor_mixing_update(int armed, float state_vec[]);

extern float attitude_commands[3]; // [roll_torque, pitch_torque, yaw_torque] in range (-1,1)
extern float thrust_command;       // normalized throttle output [0, 1]


#endif /* LIBRARIES_CONTROLLERS_H_ */
