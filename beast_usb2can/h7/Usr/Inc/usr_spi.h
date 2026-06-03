#ifndef USB2CAN_SPI_H_
#define USB2CAN_SPI_H_

#define CHIP_NUMBERS 3u
#define CHIP_CTRL_MOTOR_NUMBERS 6u
#define SPI_CMD_SIZE 56u
#define SPI_CMD_CHK_SIZE 13U
#define SPI_DATA_SIZE 76u
#define SPI_DATA_CHK_SIZE 18U

#define SPI_CMD_ALIGNMENT_ADD (32 - (SPI_CMD_SIZE % 32))
#define SPI_DATA_ALIGNMENT_ADD (32 - (SPI_DATA_SIZE % 32))

#include "spi.h"
#include "stdint.h"

// no packed because all 4 bytes data
// 128 + 4
//typedef struct _SPI_CMD {
//  float p_abad[2];
//  float p_hip[2];
//  float p_knee[2];
//  float v_abad[2];
//  float v_hip[2];
//  float v_knee[2];
//  float kp_abad[2];
//  float kp_hip[2];
//  float kp_knee[2];
//  float kd_abad[2];
//  float kd_hip[2];
//  float kd_knee[2];
//  float t_abad[2];
//  float t_hip[2];
//  float t_knee[2];
//  uint32_t flag_cmd[1];
//  uint32_t checksum_tx_message;
//} SPI_CMD_TypeDef;

//typedef struct _SPI_DATA {
//  float p_abad[2];
//  float p_hip[2];
//  float p_knee[2];
//  float v_abad[2];
//  float v_hip[2];
//  float v_knee[2];
//  float t_abad[2];
//  float t_hip[2];
//  float t_knee[2];
//  float uq_abad[2];
//  float uq_hip[2];
//  float uq_knee[2];
//  float ud_abad[2];
//  float ud_hip[2];
//  float ud_knee[2];
//  uint32_t flag_data[1];
//  uint32_t checksum_rx_message;
//} SPI_DATA_TypeDef;

// can cmd message.
typedef struct _Motor_Messgage_CMD{
  uint8_t buffer[8];
} MOTOR_MESSAGE_CMD_T;

typedef struct _Motor_Messgage_DATA{
  uint8_t buffer[12];
} MOTOR_MESSAGE_DATA_T;

// 6*8+8 = 56
typedef struct _SPI_CMD {
  MOTOR_MESSAGE_CMD_T motor_cmd[6];
  uint32_t flag_cmd[1];
  uint32_t checksum_tx_message;
} SPI_CMD_TypeDef;

typedef struct _SPI_DATA {
  MOTOR_MESSAGE_DATA_T motor_data[6];
  uint32_t checksum_rx_message;
} SPI_DATA_TypeDef;

typedef union _SPI_CMD_U {
  SPI_CMD_TypeDef spi_cmd;
  uint8_t spi_cmd_buff[SPI_CMD_SIZE];
} SPI_CMD_U;

typedef union _SPI_DATA_U {
  SPI_DATA_TypeDef spi_data;
  uint8_t spi_data_buff[SPI_DATA_SIZE];
} SPI_DATA_U;

typedef struct SPI_PROTOCAL {
  SPI_HandleTypeDef* Spi_Channel;
  GPIO_TypeDef* SPI_CS_Port;
  uint16_t SPI_CS_Pin;
  SPI_CMD_U spi_cmd_u;
  SPI_DATA_U spi_data_u;
} SPI_PROTOCAL_TypeDef;

void init_spi_protocals(void);
SPI_PROTOCAL_TypeDef* get_spi_protocal(uint8_t i);

#endif