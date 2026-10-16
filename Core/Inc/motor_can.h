#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include "can.h"

HAL_StatusTypeDef MotorCan_Init(void);
void MotorCan_Process(void);
void MotorCan_OnRx(const CAN_RxHeaderTypeDef *header,
                   const uint8_t data[8]);

#endif /* MOTOR_CAN_H */
