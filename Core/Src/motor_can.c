#include "motor_can.h"
#include "motor_control.h"

#define NODE_ID               1U
#define CONTROL_ID            (0x100U + NODE_ID)
#define STATUS_ID             (0x180U + NODE_ID)
#define POSITION_ID           (0x200U + NODE_ID)
#define HEARTBEAT_ID          (0x700U + NODE_ID)
#define PROTOCOL_VERSION      1U
#define COMMAND_TIMEOUT_MS    300U
#define STATUS_PERIOD_MS      100U
#define HEARTBEAT_PERIOD_MS   500U

static volatile uint32_t command_tick;
static volatile uint8_t command_valid;
static volatile uint8_t sequence;
static volatile uint8_t timeout_fault;
static uint32_t status_tick;
static uint32_t heartbeat_tick;

static int16_t ReadInt16(const uint8_t *data)
{
  return (int16_t)((uint16_t)data[0] |
                   ((uint16_t)data[1] << 8));
}

static void WriteInt16(uint8_t *data, int16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)((uint16_t)value >> 8);
}

static void WriteInt32(uint8_t *data, int32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)((uint32_t)value >> 8);
  data[2] = (uint8_t)((uint32_t)value >> 16);
  data[3] = (uint8_t)((uint32_t)value >> 24);
}

static HAL_StatusTypeDef SendFrame(uint16_t id, uint8_t data[8])
{
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
  {
    return HAL_BUSY;
  }

  header.StdId = id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8U;

  return HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);
}

HAL_StatusTypeDef MotorCan_Init(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (uint16_t)(CONTROL_ID << 5);
  filter.FilterMaskIdHigh = (uint16_t)(0x7FFU << 5);
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CAN_ActivateNotification(
      &hcan,
      CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF | CAN_IT_ERROR);
}

void MotorCan_OnRx(const CAN_RxHeaderTypeDef *header,
                   const uint8_t data[8])
{
  if ((header == NULL) || (data == NULL) ||
      (header->IDE != CAN_ID_STD) ||
      (header->RTR != CAN_RTR_DATA) ||
      (header->StdId != CONTROL_ID) ||
      (header->DLC != 8U) ||
      (data[7] != PROTOCOL_VERSION))
  {
    return;
  }

  if ((data[1] & 0x02U) != 0U)
  {
    timeout_fault = 0U;
  }

  if ((data[0] == 1U) && ((data[1] & 0x01U) != 0U))
  {
    MotorControl_SetTargets(ReadInt16(&data[2]),
                            ReadInt16(&data[4]));
    MotorControl_Enable(1U);
    command_valid = 1U;
  }
  else
  {
    MotorControl_Enable(0U);
    command_valid = 0U;
  }

  sequence = data[6];
  command_tick = HAL_GetTick();
}

void MotorCan_Process(void)
{
  uint32_t now = HAL_GetTick();
  MotorControl_State state;

  if ((command_valid != 0U) &&
      ((uint32_t)(now - command_tick) > COMMAND_TIMEOUT_MS))
  {
    MotorControl_Enable(0U);
    timeout_fault = 1U;
    command_valid = 0U;
  }

  if ((uint32_t)(now - status_tick) >= STATUS_PERIOD_MS)
  {
    uint8_t data[8] = {0};

    MotorControl_GetState(&state);
    WriteInt16(&data[0], state.speed_a_x10);
    WriteInt16(&data[2], state.speed_b_x10);
    data[4] = state.enabled;
    data[5] = timeout_fault;
    data[6] = sequence;
    data[7] = PROTOCOL_VERSION;

    if (SendFrame(STATUS_ID, data) == HAL_OK)
    {
      status_tick = now;
    }
  }

  if ((uint32_t)(now - heartbeat_tick) >= HEARTBEAT_PERIOD_MS)
  {
    uint8_t position_data[8] = {0};

    MotorControl_GetState(&state);
    WriteInt32(&position_data[0], state.count_a);
    WriteInt32(&position_data[4], state.count_b);

    if (SendFrame(POSITION_ID, position_data) == HAL_OK)
    {
      uint8_t heartbeat_data[8] = {0};

      heartbeat_data[0] = state.enabled;
      heartbeat_data[1] = timeout_fault;
      heartbeat_data[7] = PROTOCOL_VERSION;
      (void)SendFrame(HEARTBEAT_ID, heartbeat_data);
      heartbeat_tick = now;
    }
  }
}
