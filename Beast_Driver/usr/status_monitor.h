#ifndef Status_Monitor_H_
#define Status_Monitor_H_

#include "stdint.h"

typedef void (*Set_Channel)(void);
typedef void (*Show_Can_ID)(void);
typedef void (*Update_Board)(void);

typedef struct Board_Status
{
    float voltage_vrefint_portion;
    float V_bus_;
    float V_ins_circle_;
    float V_limit_;
    float R_value;
    float Temp_coil_;
    float V_CC;
    float board_temp_;
    int32_t temp_value;
    int32_t board_temp_value;
    int32_t v_value;
    Set_Channel pfct_set_channel;
    Show_Can_ID pfct_show_id;
    Update_Board pfct_update_board;
} Board_Status_t;

void Init_Board_Status(void);
float get_vreint_portion(void);
Board_Status_t *get_board_status_handle(void);
#endif