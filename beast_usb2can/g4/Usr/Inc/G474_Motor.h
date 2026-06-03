#ifndef G474_MOTOR_H_
#define G474_MOTOR_H_

#include "stdint.h"

#define SPI_CMD_SIZE 56u
#define SPI_CMD_CHK_SIZE 13U
#define SPI_DATA_SIZE 76u
#define SPI_DATA_CHK_SIZE 18U

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

#define KI_MIN -150.0f
#define KI_MAX 150.0f

#define UQ_MIN -40.f
#define UQ_MAX 40.f
#define UD_MIN -40.f
#define UD_MAX 40.f

#define ACC_MIN -500.0f
#define ACC_MAX 500.0f
#define TEMP_MIN 0.0f
#define TEMP_MAX 150.0f

typedef struct _Motor_Messgage_CMD {
  uint8_t buffer[8];
} MOTOR_MESSAGE_CMD_T;

typedef struct _Motor_Monitor_Status {
  float p_data_;
  float v_data_;
  float tor_data_;
  float uq_;
  float ud_;
  float acc_;
  float coil_temp_;
  uint8_t work_bit_;
} MOTOR_MONITOR_T;

typedef struct _Motor_Messgage_DATA {
  uint8_t buffer[12];
} MOTOR_MESSAGE_DATA_T;

typedef struct _SPI_CMD {
  MOTOR_MESSAGE_CMD_T motor_cmd[6];
  uint32_t flag_cmd[1];
  uint32_t checksum_rx_message;
} SPI_CMD_TypeDef;

typedef struct _SPI_DATA {
  MOTOR_MESSAGE_DATA_T motor_data[6];
  uint32_t checksum_tx_message;
} SPI_DATA_TypeDef;

// 48 + 8 = 56 original
// 12 * 6 + 8 = 80
typedef union _SPI_CMD_U {
  SPI_CMD_TypeDef spi_cmd;
  uint8_t spi_cmd_buff[SPI_CMD_SIZE];
} MOTOR_CMDs_U;

typedef union _SPI_DATA_U {
  SPI_DATA_TypeDef spi_data;
  uint8_t spi_data_buff[SPI_DATA_SIZE];
} MOTOR_DATAs_U;

typedef enum control_mode {
  null_ctrl = 0,
  mit_ctrl,
  vel_ctrl,
  normal_ctrl,
  pos_ctrl,
  zero,
  cali,
  disable
} CONTROL_MODE_T;

typedef struct Motors_Manager {
  void (*error_check)(void);
  void (*pause_check)(void);
  void (*send_msgs_id1)(void);
  void (*send_msgs_id2)(void);
  void (*send_msgs_id3)(void);
  void (*receive_msgs)(uint8_t can_bus_id);

  MOTOR_CMDs_U motor_cmds_;
  MOTOR_DATAs_U motor_datas_;
  CONTROL_MODE_T control_mode_;
} MOTORs_MANAGER_t;

void init_motors_manager(void);
MOTORs_MANAGER_t* get_motors_manager(void);
SPI_CMD_TypeDef* get_motors_cmd(void);
void manager_task(void);

static inline uint32_t data_checksum(uint32_t* data_to_check, uint32_t check_length) {
  uint32_t t = 0;
  for (int i = 0; i < check_length; i++) {
    t = t ^ data_to_check[i];
  }
  return t;
}

static inline float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span   = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void trick_data(void);

#endif