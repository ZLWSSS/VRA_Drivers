#ifndef FSM_H_
#define FSM_H_

#include "stdint.h"

typedef void (*State_Exit)(void);
typedef void (*State_Enter)(void);
typedef void (*State_Run)(void);

typedef struct FSM_State
{
    State_Enter pfct_state_enter;
    State_Exit pfct_state_exit;
    State_Run pfct_state_run;
    struct FSM_State *next_state;
} FSM_State_t;

void ram_main(void);
void current_loop(void);
void iq_cmd_update_loop(void);
void can_message_pending(void);
void Sample_Encoder(void);
void Cmd_Fresh_Check(uint32_t timer);
void update_voltage_temperature(void);
void dac_out_start(void);
extern uint8_t initialized;
#endif