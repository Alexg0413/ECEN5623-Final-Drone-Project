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
#define KP_ROLL     0.0f
#define KI_ROLL     0.0f
#define KD_ROLL     0.0f
#define IMAX_ROLL   1.0f

#define KP_PITCH    0.0f
#define KI_PITCH    0.0f
#define KD_PITCH    0.0f
#define IMAX_PITCH  1.0f

#define KP_ROLL_RATE     0.0f
#define KI_ROLL_RATE     0.0f
#define KD_ROLL_RATE     0.0f
#define IMAX_ROLL_RATE   1.0f

#define KP_PITCH_RATE    0.0f
#define KI_PITCH_RATE    0.0f
#define KD_PITCH_RATE    0.0f
#define IMAX_PITCH_RATE  1.0f

#define KP_YAW_RATE    0.0f
#define KI_YAW_RATE     0.0f
#define KD_YAW_RATE     0.0f
#define IMAX_YAW_RATE   1.0f

///////// used for z-pos hold controller 
#define KP_CLIMB_RATE   0.0f
#define KI_CLIMB_RATE   0.0f
#define KD_CLIMB_RATE   0.0f
#define IMAX_CLIMB_RATE 1.0f

#define KP_CLIMB_ACCEL   0.0f
#define KI_CLIMB_ACCEL   0.0f
#define KD_CLIMB_ACCEL   0.0f
#define IMAX_CLIMB_ACCEL 1.0f

// Derivative low-pass cutoff frequency (Hz) applied to all axes
#define FCUT_HZ     30.0f

// Initialize all PID controllers with above gains
void attitude_controllers_init(void);

// Read state_vec / input_vec, run all four PIDs, write results to output_vec
void attitude_controllers_update(float state_vec[], float input_vec[], int output_vec[]);

#endif /* LIBRARIES_CONTROLLERS_H_ */
