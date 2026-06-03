#include "G474_Can.h"
#include "G474_Motor.h"
#include "fdcan.h"
#include "usr_delay.h"

CAN_MANAGER_t can_manager;
// header for data
FDCAN_TxHeaderTypeDef can_tx_header_data;
// header for rtr
FDCAN_TxHeaderTypeDef can_tx_header_rtr;
void fdcan_can_init(void);
SPI_CMD_TypeDef* motor_cmds;
uint8_t void_vec[8];

CAN_MANAGER_t* get_can_manager_handle(void) {
  return &can_manager;
}

// send msgs: id 0 / 3
static void Send_Msgs_Normal(uint8_t id) {
  can_tx_header_data.Identifier = id;

  uint8_t cmd_ind0 = id - 1;
  uint8_t cmd_ind1 = cmd_ind0 + 3;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header_data, motor_cmds->motor_cmd[cmd_ind0].buffer);
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can_tx_header_data, motor_cmds->motor_cmd[cmd_ind1].buffer);
}

static void Send_Msgs_Enable_Mit(uint8_t id) {
  can_tx_header_rtr.Identifier = MIT_MODE_HEADER | id;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header_rtr, void_vec);
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can_tx_header_rtr, void_vec);
}

static void Send_Msgs_Enable_Vel(uint8_t id) {
  can_tx_header_rtr.Identifier = VEL_MODE_HEADER | id;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header_rtr, void_vec);
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can_tx_header_rtr, void_vec);
}

static void Send_Msgs_Enable_Pos(uint8_t id) {
  can_tx_header_rtr.Identifier = POSITION_MODE_HEADER | id;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header_rtr, void_vec);
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can_tx_header_rtr, void_vec);
}

static void Send_Msgs_Dis(uint8_t id) {
  can_tx_header_rtr.Identifier = DISABLE_HEADER | id;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header_rtr, void_vec);
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can_tx_header_rtr, void_vec);
}

// 1 2 3 4 5 6
static void Send_Msgs_Zero(uint8_t id) {
  int id_cmd     = can_manager.zero_motor_id_;
  int select_can = (id_cmd - 1) / 3;
  int motor_id   = (id_cmd - 1) % 3 + 1;

  if (motor_id == id) {
    can_tx_header_rtr.Identifier = ZERO_HEADER | id;
    if (select_can == 0) {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header_rtr, void_vec);
    } else {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can_tx_header_rtr, void_vec);
    }
  }
}

static void Send_Msgs_Cali(uint8_t id) {
  int id_cmd     = can_manager.cali_motor_id_;
  int select_can = (id_cmd - 1) / 3;
  int motor_id   = (id_cmd - 1) % 3 + 1;

  if (motor_id == id) {
    can_tx_header_rtr.Identifier = CALI_HEADER | id;
    if (select_can == 0) {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header_rtr, void_vec);
    } else {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &can_tx_header_rtr, void_vec);
    }
  }
}

static void FDCAN_STB(void) {
  HAL_GPIO_WritePin(FDCAN2_STB_GPIO_Port, FDCAN2_STB_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FDCAN1_STB_GPIO_Port, FDCAN1_STB_Pin, GPIO_PIN_RESET);
}

static void FDCAN_HIZ(void) {
  HAL_GPIO_WritePin(FDCAN2_STB_GPIO_Port, FDCAN2_STB_Pin, GPIO_PIN_SET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FDCAN1_STB_GPIO_Port, FDCAN1_STB_Pin, GPIO_PIN_SET);
}

void fdcan_can_init(void) {
  FDCAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  can_filter_st.FilterID1    = 0x000;
  can_filter_st.FilterID2    = 0x0000;
  can_filter_st.FilterIndex  = 0;
  can_filter_st.FilterType   = FDCAN_FILTER_MASK;
  can_filter_st.IdType       = FDCAN_STANDARD_ID;

  // can1 --> fifo0
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_st) != HAL_OK) {
    Error_Handler();
  }

  can_filter_st.FilterID1    = 0x000;
  can_filter_st.FilterID2    = 0x0000;
  can_filter_st.FilterIndex  = 1;
  can_filter_st.FilterType   = FDCAN_FILTER_MASK;
  can_filter_st.IdType       = FDCAN_STANDARD_ID;
  can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter_st) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0)) {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigInterruptLines(&hfdcan2, FDCAN_IT_GROUP_RX_FIFO1, FDCAN_INTERRUPT_LINE1)) {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigGlobalFilter(
          &hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigGlobalFilter(
          &hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
    Error_Handler();
  }

  can_tx_header_data.DataLength          = FDCAN_DLC_BYTES_8;
  can_tx_header_data.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  can_tx_header_data.MessageMarker       = 0;
  can_tx_header_data.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  can_tx_header_data.IdType              = FDCAN_STANDARD_ID;
  can_tx_header_data.TxFrameType         = FDCAN_DATA_FRAME;

  can_tx_header_rtr.DataLength          = FDCAN_DLC_BYTES_8;
  can_tx_header_rtr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  can_tx_header_rtr.MessageMarker       = 0;
  can_tx_header_rtr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  can_tx_header_rtr.IdType              = FDCAN_STANDARD_ID;
  can_tx_header_rtr.TxFrameType         = FDCAN_REMOTE_FRAME;

#ifdef FDCAN
  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 26, 0) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2, 26, 0) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2) != HAL_OK) {
    Error_Handler();
  }

  can_tx_header_data.BitRateSwitch = FDCAN_BRS_ON;
  can_tx_header_data.FDFormat      = FDCAN_FD_CAN;
  can_tx_header_rtr.BitRateSwitch  = FDCAN_BRS_ON;
  can_tx_header_rtr.FDFormat       = FDCAN_FD_CAN;
#endif

#ifdef CLASSICCAN
  can_tx_header_data.BitRateSwitch = FDCAN_BRS_OFF;
  can_tx_header_data.FDFormat      = FDCAN_CLASSIC_CAN;
  can_tx_header_rtr.BitRateSwitch  = FDCAN_BRS_OFF;
  can_tx_header_rtr.FDFormat       = FDCAN_CLASSIC_CAN;
#endif

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
    Error_Handler();
  }
}

void init_can_manager(void) {
  fdcan_can_init();
  can_manager.send_msgs_normal     = Send_Msgs_Normal;
  can_manager.send_msgs_enable_mit = Send_Msgs_Enable_Mit;
  can_manager.send_msgs_enable_vel = Send_Msgs_Enable_Vel;
  can_manager.send_msgs_enable_pos = Send_Msgs_Enable_Pos;
  can_manager.send_msgs_dis        = Send_Msgs_Dis;
  can_manager.send_msgs_zero       = Send_Msgs_Zero;
  can_manager.send_msgs_cali       = Send_Msgs_Cali;
  can_manager.fdcan_hiz            = FDCAN_HIZ;
  can_manager.fdcan_stb            = FDCAN_STB;

  motor_cmds = get_motors_cmd();
}

void disable_motors_once_setup(void) {
  can_manager.send_msgs_dis(1);
  delay_ms(1);
  can_manager.send_msgs_dis(2);
  delay_ms(1);
  can_manager.send_msgs_dis(3);
}