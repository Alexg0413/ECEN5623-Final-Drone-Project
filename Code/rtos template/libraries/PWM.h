#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>
#include <stdbool.h>

// PWM input capture data
extern volatile uint32_t pulse_width[6];

// Init functions
void PWM_Input_Init(void);
void PWM_Output_Init(void);
void Motor_Update(int *cmd);

// Timer ISR handlers
void Timer5AIntHandler(void);
void Timer5BIntHandler(void);
void Timer3AIntHandler(void);
void Timer3BIntHandler(void);
void Timer4AIntHandler(void);
void Timer4BIntHandler(void);

#endif

