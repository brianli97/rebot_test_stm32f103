#include "motor_control.h"
#include "tim.h"

#define CONTROL_PERIOD_MS   50U
#define PWM_MAX             3599
#define TARGET_MAX          1300
#define COUNTS_PER_REV      2024.0f
#define FILTER_ALPHA        0.3f
#define SPEED_KP            5.0f
#define SPEED_KI            0.5f
#define PWM_PER_RPM         30.0f
#define PWM_STEP            80

static MotorControl_State motor_state;
static uint16_t encoder_a_last;
static uint16_t encoder_b_last;
static uint32_t control_tick;
static float motor_a_rpm;
static float motor_b_rpm;
static float motor_a_integral;
static float motor_b_integral;

static float ClampFloat(float value, float minimum, float maximum)
{
  if (value > maximum)
  {
    return maximum;
  }

  if (value < minimum)
  {
    return minimum;
  }

  return value;
}

static int16_t ClampTarget(int16_t target)
{
  if (target > TARGET_MAX)
  {
    return TARGET_MAX;
  }

  if (target < -TARGET_MAX)
  {
    return -TARGET_MAX;
  }

  return target;
}

static int16_t LimitPwmStep(int16_t requested, int16_t previous)
{
  if (requested > (int16_t)(previous + PWM_STEP))
  {
    return previous + PWM_STEP;
  }

  if (requested < (int16_t)(previous - PWM_STEP))
  {
    return previous - PWM_STEP;
  }

  return requested;
}

static void SetMotorPwm(GPIO_TypeDef *in1_port,
                        uint16_t in1_pin,
                        GPIO_TypeDef *in2_port,
                        uint16_t in2_pin,
                        uint32_t channel,
                        int16_t pwm)
{
  if (pwm > PWM_MAX)
  {
    pwm = PWM_MAX;
  }
  else if (pwm < -PWM_MAX)
  {
    pwm = -PWM_MAX;
  }

  HAL_GPIO_WritePin(in1_port, in1_pin,
                    (pwm > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(in2_port, in2_pin,
                    (pwm < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim8, channel,
                        (uint16_t)((pwm < 0) ? -pwm : pwm));
}

static int16_t SpeedPiUpdate(float target,
                             float measured,
                             float delta_time,
                             float *integral)
{
  float error;
  float output;

  if ((target > -0.05f) && (target < 0.05f))
  {
    *integral = 0.0f;
    return 0;
  }

  error = target - measured;
  *integral = ClampFloat(*integral + error * delta_time,
                         -(float)PWM_MAX / SPEED_KI,
                         (float)PWM_MAX / SPEED_KI);
  output = target * PWM_PER_RPM +
           SPEED_KP * error +
           SPEED_KI * (*integral);

  if (target > 0.0f)
  {
    output = ClampFloat(output, 0.0f, (float)PWM_MAX);
  }
  else
  {
    output = ClampFloat(output, -(float)PWM_MAX, 0.0f);
  }

  return (int16_t)output;
}

HAL_StatusTypeDef MotorControl_Init(void)
{
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port,
                    MOTOR_STBY_Pin,
                    GPIO_PIN_RESET);
  control_tick = HAL_GetTick();

  return HAL_OK;
}

void MotorControl_Enable(uint8_t enable)
{
  motor_state.enabled = (enable != 0U) ? 1U : 0U;
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port,
                    MOTOR_STBY_Pin,
                    motor_state.enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);

  if (motor_state.enabled == 0U)
  {
    motor_state.target_a_x10 = 0;
    motor_state.target_b_x10 = 0;
    motor_state.pwm_a = 0;
    motor_state.pwm_b = 0;
    motor_a_integral = 0.0f;
    motor_b_integral = 0.0f;

    SetMotorPwm(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin,
                MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin,
                TIM_CHANNEL_1, 0);
    SetMotorPwm(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin,
                MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin,
                TIM_CHANNEL_2, 0);
  }
}

void MotorControl_SetTargets(int16_t a_rpm_x10,
                             int16_t b_rpm_x10)
{
  motor_state.target_a_x10 = ClampTarget(a_rpm_x10);
  motor_state.target_b_x10 = ClampTarget(b_rpm_x10);
}

void MotorControl_Process(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed = now - control_tick;
  uint16_t encoder_a_now;
  uint16_t encoder_b_now;
  int16_t encoder_a_delta;
  int16_t encoder_b_delta;
  float delta_time;
  float motor_a_raw_rpm;
  float motor_b_raw_rpm;

  if (elapsed < CONTROL_PERIOD_MS)
  {
    return;
  }

  encoder_a_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  encoder_b_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  encoder_a_delta = (int16_t)(encoder_a_now - encoder_a_last);
  encoder_b_delta = (int16_t)(encoder_b_now - encoder_b_last);
  encoder_a_last = encoder_a_now;
  encoder_b_last = encoder_b_now;
  control_tick = now;

  motor_state.count_a += encoder_a_delta;
  motor_state.count_b += encoder_b_delta;
  delta_time = (float)elapsed / 1000.0f;
  motor_a_raw_rpm = -1.0f * encoder_a_delta * 60.0f /
                    (COUNTS_PER_REV * delta_time);
  motor_b_raw_rpm = -1.0f * encoder_b_delta * 60.0f /
                    (COUNTS_PER_REV * delta_time);
  motor_a_rpm += FILTER_ALPHA * (motor_a_raw_rpm - motor_a_rpm);
  motor_b_rpm += FILTER_ALPHA * (motor_b_raw_rpm - motor_b_rpm);
  motor_state.speed_a_x10 = (int16_t)(motor_a_rpm * 10.0f);
  motor_state.speed_b_x10 = (int16_t)(motor_b_rpm * 10.0f);

  if (motor_state.enabled != 0U)
  {
    int16_t requested_a;
    int16_t requested_b;

    requested_a = SpeedPiUpdate(motor_state.target_a_x10 / 10.0f,
                                motor_a_rpm,
                                delta_time,
                                &motor_a_integral);
    requested_b = SpeedPiUpdate(motor_state.target_b_x10 / 10.0f,
                                motor_b_rpm,
                                delta_time,
                                &motor_b_integral);
    motor_state.pwm_a = LimitPwmStep(requested_a, motor_state.pwm_a);
    motor_state.pwm_b = LimitPwmStep(requested_b, motor_state.pwm_b);
  }

  SetMotorPwm(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin,
              MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin,
              TIM_CHANNEL_1,
              motor_state.enabled ? motor_state.pwm_a : 0);
  SetMotorPwm(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin,
              MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin,
              TIM_CHANNEL_2,
              motor_state.enabled ? motor_state.pwm_b : 0);
}

void MotorControl_GetState(MotorControl_State *state)
{
  if (state == NULL)
  {
    return;
  }

  __disable_irq();
  *state = motor_state;
  __enable_irq();
}
