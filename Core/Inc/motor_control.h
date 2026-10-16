#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"

typedef struct
{
  int32_t count_a;
  int32_t count_b;
  int16_t speed_a_x10;
  int16_t speed_b_x10;
  int16_t target_a_x10;
  int16_t target_b_x10;
  int16_t pwm_a;
  int16_t pwm_b;
  uint8_t enabled;
} MotorControl_State;

HAL_StatusTypeDef MotorControl_Init(void);
void MotorControl_Process(void);
void MotorControl_Enable(uint8_t enable);
void MotorControl_SetTargets(int16_t a_rpm_x10,
                             int16_t b_rpm_x10);
void MotorControl_GetState(MotorControl_State *state);

#endif /* MOTOR_CONTROL_H */
