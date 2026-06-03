#ifndef G474_CAN_H_
#define G474_CAN_H_

#include "stdint.h"

#define ENABLE_HEADER 0x200
#define DISABLE_HEADER 0x210
#define MIT_MODE_HEADER 0x010
#define VEL_MODE_HEADER 0x020
#define POSITION_MODE_HEADER 0x030
#define ZERO_HEADER 0x230
#define CALI_HEADER 0x220

typedef struct CAN_Manager {
  void (*send_msgs_normal)(uint8_t id);
  void (*send_msgs_enable_mit)(uint8_t id);
  void (*send_msgs_enable_vel)(uint8_t id);
  void (*send_msgs_enable_pos)(uint8_t id);
  void (*send_msgs_dis)(uint8_t id);
  void (*send_msgs_zero)(uint8_t id);
  void (*send_msgs_cali)(uint8_t id);
  void (*fdcan_stb)(void);
  void (*fdcan_hiz)(void);

  uint32_t zero_motor_id_;    // only for zero and cali
  uint32_t cali_motor_id_;
} CAN_MANAGER_t;

CAN_MANAGER_t* get_can_manager_handle(void);
void init_can_manager(void);
void disable_motors_once_setup(void);

#endif