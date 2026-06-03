#include "can_handler.h"
#include "config.h"
#include "fdcan.h"

CAN_Handler_t can_handler_;

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void get_control_cmd(uint8_t *cmd_data)
{
    int p_int = (cmd_data[0] << 8) | cmd_data[1];
    int v_int = (cmd_data[2] << 4) | (cmd_data[3] >> 4);
    int kp_int = ((cmd_data[3] & 0xF) << 8) | cmd_data[4];
    int kd_int = (cmd_data[5] << 4) | (cmd_data[6] >> 4);
    int t_int = ((cmd_data[6] & 0xF) << 8) | cmd_data[7];

    can_handler_.cmd_p_target = uint_to_float(p_int, P_MIN, P_MAX, 16);
    can_handler_.cmd_v_target = uint_to_float(v_int, V_MIN, V_MAX, 12);
    can_handler_.cmd_kp = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
    can_handler_.cmd_kd = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
    can_handler_.cmd_t_target = uint_to_float(t_int, TOR_MIN, TOR_MAX, 12);
}

static void reply_data(float p, float v, float t, float uq, float ud)
{
    int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(t, TOR_MIN, TOR_MAX, 12);
    // TODO check the filter the uq ud data.
    uq = uq * LPF_U_Alpha + can_handler_.uq_last * LPF_U_Beta;
    ud = ud * LPF_U_Alpha + can_handler_.ud_last * LPF_U_Beta;
    can_handler_.uq_last = uq;
    can_handler_.ud_last = ud;
    int uq_int = float_to_uint(uq, UQ_MIN, UQ_MAX, 12);
    int ud_int = float_to_uint(ud, UD_MIN, UD_MAX, 12);

    can_handler_.data_buf[0] = p_int >> 8;
    can_handler_.data_buf[1] = p_int & 0xFF;
    can_handler_.data_buf[2] = v_int >> 4;
    can_handler_.data_buf[3] = ((v_int & 0xF) << 4) + (t_int >> 8);
    can_handler_.data_buf[4] = t_int & 0xFF;
    can_handler_.data_buf[5] = uq_int >> 4;
    can_handler_.data_buf[6] = ((uq_int & 0xF) << 4) | ud_int >> 8;
    can_handler_.data_buf[7] = ud_int & 0xFF;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &(can_handler_.can_txheader), can_handler_.data_buf);
}

static void reset_cmd(void)
{
    can_handler_.cmd_kp = 0;
    can_handler_.cmd_kd = 0;
    can_handler_.cmd_p_target = can_handler_.cmd_p_last = 0;
    can_handler_.cmd_v_last = can_handler_.cmd_v_target = 0;
    can_handler_.cmd_t_last = can_handler_.cmd_t_target = 0;
}

void Can_Handler_Init(void)
{
    FDCAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterIndex = 0;
    can_filter_st.IdType = FDCAN_STANDARD_ID;
    can_filter_st.FilterType = FDCAN_FILTER_MASK;
    can_filter_st.FilterID1 = CAN_ID;
    can_filter_st.FilterID2 = 0x00F ;
    //can_filter_st.FilterID1 = 0x000; // receive all
    //can_filter_st.FilterID2 = 0x000;
    can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_st) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0))
    {
        Error_Handler();
    }
    // TODO reject non-match id.
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
#ifdef CAN_FDCAN
    // prescalar * TQ1.
    if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 28, 0) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
#endif

    can_handler_.can_id = CAN_ID;
    can_handler_.pfct_get_control_cmd = get_control_cmd;
    can_handler_.pfct_reply_data = reply_data;
    can_handler_.pfct_can_reset_cmd = reset_cmd;
    can_handler_.can_txheader.DataLength = FDCAN_DLC_BYTES_8; // remove id byte
#ifdef CAN_CLASSIC
    can_handler_.can_txheader.BitRateSwitch = FDCAN_BRS_OFF;
    can_handler_.can_txheader.FDFormat = FDCAN_CLASSIC_CAN;
#endif
#ifdef CAN_FDCAN
    can_handler_.can_txheader.BitRateSwitch = FDCAN_BRS_ON;
    can_handler_.can_txheader.FDFormat = FDCAN_FD_CAN;
#endif
    can_handler_.can_txheader.MessageMarker = 0;
    can_handler_.can_txheader.Identifier = CAN_ID;
    can_handler_.can_txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    can_handler_.can_txheader.IdType = FDCAN_STANDARD_ID;
    can_handler_.can_txheader.TxFrameType = FDCAN_DATA_FRAME;
    can_handler_.can_txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    //NOTE prepare the chip
    HAL_GPIO_WritePin(FDCAN_STB_GPIO_Port, FDCAN_STB_Pin, GPIO_PIN_RESET);
}

// TODO Try whether const could reply data?
CAN_Handler_t *get_can_handler_t(void)
{
    return &can_handler_;
}
