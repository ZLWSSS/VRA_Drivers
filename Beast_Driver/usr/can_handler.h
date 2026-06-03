#ifndef CAN_HANDLER_H_
#define CAN_HANDLER_H_

#include "fdcan.h"
#include "motor_config.h"

// NOTE: these parameters should match the scope of h7
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define TOR_MIN -150.0f
#define TOR_MAX 150.0f
#define UQ_MIN -40.f
#define UQ_MAX 40.f
#define UD_MIN -40.f
#define UD_MAX 40.f

// RTR ID
#define En_MIT_MODE_ID ((0x010 | CAN_ID)) // default
#define En_VEL_MODE_ID ((0x020 | CAN_ID))
#define En_POSITION_MODE_ID ((0x030 | CAN_ID))
// #define ANTI_COGGING_CALI_ID 0x103
#define Require_Status_ID ((0x040 | CAN_ID))
#define DAMPING_MODE_ID ((0x050 | CAN_ID))
#define ENABLE_ID ((0x200 | CAN_ID))
#define DISABLE_ID ((0x210 | CAN_ID))
#define CALIBRATION_ID ((0x220 | CAN_ID))
#define ZERO_POSITION_ID ((0x230 | CAN_ID))

typedef struct CAN_Handler CAN_Handler_t;
typedef void (*Get_Control_Cmd)(uint8_t *cmd_data);
typedef void (*Reply_Data)(float p, float v, float t, float uq, float ud);
typedef void (*Can_Reset_Cmd)(void);

struct CAN_Handler
{
    uint8_t can_id;
    float cmd_p_last;
    float cmd_p_target;
    float cmd_v_last;
    float cmd_v_target;
    float cmd_t_last;
    float cmd_t_target;
    float cmd_kp;
    float cmd_kd;
    float uq_last;
    float ud_last; // low-pass filter for uq_ud
    uint8_t data_buf[8];
    uint8_t cmd_buf[8];
    FDCAN_TxHeaderTypeDef can_txheader;
    Get_Control_Cmd pfct_get_control_cmd;
    Reply_Data pfct_reply_data;
    Can_Reset_Cmd pfct_can_reset_cmd;
};

void Can_Handler_Init(void);
CAN_Handler_t *get_can_handler_t(void);

#endif