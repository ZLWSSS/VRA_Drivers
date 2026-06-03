#include "G474_Motor.h"
#include "G474_Can.h"
#include "fdcan.h"
#include "spi.h"
#include "stm32g4xx_ll_gpio.h"
#include "string.h"
#include "usr_delay.h"
#ifdef fake_data
#include "math.h"
#endif

// NOTE: the CCM_RAM_DATA here is remap of CCM_RAM. DAM can only access to
// CCM_RAM_Data to this area.
MOTORs_MANAGER_t __attribute((section(".CCM_RAM_Data"))) motors_manager;
CAN_MANAGER_t *local_can_manager;

SPI_CMD_TypeDef motors_cmd;
SPI_DATA_TypeDef motors_data;
#ifdef DEBUG
MOTOR_MONITOR_T motor_data_monitor;
#endif

uint8_t dma_transmitting = 0;
uint32_t dma_success_update = 0;

// used for msgs receiving
FDCAN_RxHeaderTypeDef rx_header;
uint8_t motor_msg[12]; // 12 bit
int motor_msg_length = 12;
int msg_id = 0;
uint32_t ticker = 0;

uint8_t one_step_locker1 = 1;
uint8_t one_step_locker2 = 1;
uint8_t one_step_locker3 = 1;

#define ENABLE_SHIFT 0
#define DISABLE_SHIFT 3
#define CTRL_SHIFT 6
#define ZERO_SHIFT 7
#define CALI_SHIFT 10

#define ID1_SHIFT 0
#define ID2_SHIFT 1
#define ID3_SHIFT 2

#define CMD_ZERO_SHIFT 5
#define CMD_CALI_SHIFT 10

#define PAUSE_STATUS 0x38

// monitor motor id: 0 ~ 5
#define Number_Monitor_Motor 0

MOTORs_MANAGER_t *get_motors_manager(void)
{
    return &motors_manager;
}

SPI_CMD_TypeDef *get_motors_cmd(void)
{
    return &motors_manager.motor_cmds_.spi_cmd;
}

// 000       000         0             000         000
// cali 3 bits | zero 3 bits  | normal_ctrl | 3 disable | 3 enable.
volatile uint32_t status_bits = 0;

static void Error_Check(void)
{
}

// send msgs for id1 motors (2 motor)
// NOTE cost 1.54us, max 8.62us.
void Send_Msgs_ID1(void)
{
    ////LL_GPIO_SetOutputPin(GPIOA, LED_1_Pin | LED_2_Pin);
    CONTROL_MODE_T ctrl_mode = motors_manager.control_mode_;
    switch (ctrl_mode)
    {
    case normal_ctrl:
        local_can_manager->send_msgs_normal(1);
        break;
    case mit_ctrl:
        if (!((status_bits >> (ENABLE_SHIFT + ID1_SHIFT)) & 0x01))
        {
            local_can_manager->send_msgs_enable_mit(1);
            status_bits |= (0x01 << (ENABLE_SHIFT + ID1_SHIFT));
        }
        break;
    case vel_ctrl:
        if (!((status_bits >> (ENABLE_SHIFT + ID1_SHIFT)) & 0x01))
        {
            local_can_manager->send_msgs_enable_vel(1);
            status_bits |= (0x01 << (ENABLE_SHIFT + ID1_SHIFT));
        }
        break;
    case pos_ctrl:
        if (!((status_bits >> (ENABLE_SHIFT + ID1_SHIFT)) & 0x01))
        {
            local_can_manager->send_msgs_enable_pos(1);
            status_bits |= (0x01 << (ENABLE_SHIFT + ID1_SHIFT));
        }
        break;
    case zero:
        if (((status_bits >> (ZERO_SHIFT + ID1_SHIFT)) & 0x01) & one_step_locker1)
        {
            local_can_manager->send_msgs_zero(1);
            status_bits &= ~(1 << (ZERO_SHIFT + ID1_SHIFT));
            one_step_locker1 = 0;
        }
        break;
    case cali:
        if (((status_bits >> (CALI_SHIFT + ID1_SHIFT)) & 0x01) & one_step_locker1)
        {
            local_can_manager->send_msgs_cali(1);
            status_bits &= ~(1 << (CALI_SHIFT + ID1_SHIFT));
            one_step_locker1 = 0;
        }
        break;
    case disable:
        if (!((status_bits >> (DISABLE_SHIFT + ID1_SHIFT)) & 0x01))
        {
            local_can_manager->send_msgs_dis(1);
            status_bits |= (0x01 << (DISABLE_SHIFT + ID1_SHIFT));
        }
        break;
    case null_ctrl:
        break;
    default:
        break;
    }
    // LL_GPIO_ResetOutputPin(GPIOA, LED_1_Pin | LED_2_Pin);
}

// send msgs for id2 motors (2 motor)
// NOTE cost 1.54us
static void Send_Msgs_ID2(void)
{
    CONTROL_MODE_T ctrl_mode = motors_manager.control_mode_;
    switch (ctrl_mode)
    {
    case normal_ctrl:
        local_can_manager->send_msgs_normal(2);
        break;
    case mit_ctrl:
        if (!((status_bits >> (ENABLE_SHIFT + ID2_SHIFT)) & 0x01))
        {
            local_can_manager->send_msgs_enable_mit(2);
            status_bits |= (0x01 << (ENABLE_SHIFT + ID2_SHIFT));
        }
        break;
    case vel_ctrl:
        if (!((status_bits >> (ENABLE_SHIFT + ID2_SHIFT)) & 0x01))
        {
            local_can_manager->send_msgs_enable_vel(2);
            status_bits |= (0x01 << (ENABLE_SHIFT + ID2_SHIFT));
        }
        break;
    case pos_ctrl:
        if (!((status_bits >> (ENABLE_SHIFT + ID2_SHIFT)) & 0x01))
        {
            local_can_manager->send_msgs_enable_pos(2);
            status_bits |= (0x01 << (ENABLE_SHIFT + ID2_SHIFT));
        }
        break;
    case zero:
        if (((status_bits >> (ZERO_SHIFT + ID2_SHIFT)) & 0x01) & one_step_locker2)
        {
            local_can_manager->send_msgs_zero(2);
            status_bits &= ~(1 << (ZERO_SHIFT + ID2_SHIFT));
            one_step_locker2 = 0;
        }
        break;
    case cali:
        if (((status_bits >> (CALI_SHIFT + ID2_SHIFT)) & 0x01) & one_step_locker2)
        {
            local_can_manager->send_msgs_cali(2);
            status_bits &= ~(1 << (CALI_SHIFT + ID2_SHIFT));
            one_step_locker2 = 0;
        }
        break;
    case disable:
        if (!((status_bits >> (DISABLE_SHIFT + ID2_SHIFT)) & 0x01))
        {
            local_can_manager->send_msgs_dis(2);
            status_bits |= (0x01 << (DISABLE_SHIFT + ID2_SHIFT));
        }
        break;
    default:
        break;
    }
}

// send msgs for id3 motors (2 motor)
static void Send_Msgs_ID3(void)
{
    CONTROL_MODE_T ctrl_mode = motors_manager.control_mode_;
    switch (ctrl_mode)
    {
    case normal_ctrl:
        local_can_manager->send_msgs_normal(3);
        break;
    case mit_ctrl:
        if (!((status_bits >> (ENABLE_SHIFT + ID3_SHIFT)) & 0x01))
        {
            local_can_manager->send_msgs_enable_mit(3);
            status_bits |= (0x01 << (ENABLE_SHIFT + ID3_SHIFT));
        }
        break;
    case vel_ctrl:
        if (!((status_bits >> (ENABLE_SHIFT + ID3_SHIFT)) & 0x01))
        {
            local_can_manager->send_msgs_enable_vel(3);
            status_bits |= (0x01 << (ENABLE_SHIFT + ID3_SHIFT));
        }
        break;
    case pos_ctrl:
        if (!((status_bits >> (ENABLE_SHIFT + ID3_SHIFT)) & 0x01))
        {
            local_can_manager->send_msgs_enable_pos(3);
            status_bits |= (0x01 << (ENABLE_SHIFT + ID3_SHIFT));
        }
        break;
    case zero:
        if (((status_bits >> (ZERO_SHIFT + ID3_SHIFT)) & 0x01) & one_step_locker3)
        {
            local_can_manager->send_msgs_zero(3);
            status_bits &= ~(1 << (ZERO_SHIFT + ID3_SHIFT));
            one_step_locker3 = 0;
        }
        break;
    case cali:
        if (((status_bits >> (CALI_SHIFT + ID3_SHIFT)) & 0x01) & one_step_locker3)
        {
            local_can_manager->send_msgs_cali(3);
            status_bits &= ~(1 << (CALI_SHIFT + ID3_SHIFT));
            one_step_locker3 = 0;
        }
        break;
    case disable:
        if (!((status_bits >> (DISABLE_SHIFT + ID3_SHIFT)) & 0x01))
        {
            local_can_manager->send_msgs_dis(3);
            status_bits |= (0x01 << (DISABLE_SHIFT + ID3_SHIFT));
        }
        break;
    default:
        break;
    }
}

// NOTE max:11 us normal:2.35us
static void Receive_Msgs(uint8_t can_bus_id)
{
    // LL_GPIO_SetOutputPin(GPIOA, LED_1_Pin | LED_2_Pin);
    switch (can_bus_id)
    {
    case 1:
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, motor_msg);
        msg_id = rx_header.Identifier;
        memcpy(&motors_data.motor_data[msg_id - 1], motor_msg, motor_msg_length);
        break;
    case 2:
        HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &rx_header, motor_msg);
        msg_id = rx_header.Identifier;
        memcpy(&motors_data.motor_data[msg_id + 2], motor_msg, motor_msg_length);
        break;
    default:
        break;
    }
    // LL_GPIO_ResetOutputPin(GPIOA, LED_1_Pin | LED_2_Pin);
}

// tick 500hz
static void Pause_Check(void)
{
    if ((dma_success_update == 0) & dma_transmitting)
    {
        motors_manager.control_mode_ = null_ctrl;
        dma_transmitting = 0;
        status_bits = PAUSE_STATUS;
        one_step_locker1 = one_step_locker2 = one_step_locker3 = 1;
    }
    dma_success_update = 0;
    // if (ticker % 500 == 0) {
    //   HAL_GPIO_TogglePin(GPIOA, LED_1_Pin | LED_2_Pin);
    // }
}

// NOTE 5.4us spi tick 3000hz
uint32_t tick_count = 0;
uint32_t tick_count_times = 100;
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_CMD_TypeDef *spi_cmd_ = &motors_manager.motor_cmds_.spi_cmd;
    SPI_DATA_TypeDef *spi_data_ = &motors_manager.motor_datas_.spi_data;

    uint32_t t = data_checksum((uint32_t *)spi_cmd_, SPI_CMD_CHK_SIZE);
    if (t == spi_cmd_->checksum_rx_message)
    {
        // cmd_ok
        memcpy(&motors_cmd, spi_cmd_, sizeof(SPI_CMD_TypeDef));
        memcpy(&(motors_manager.motor_datas_.spi_data), &motors_data, sizeof(SPI_DATA_TypeDef));
        motors_manager.motor_datas_.spi_data.checksum_tx_message =
            data_checksum((uint32_t *)(spi_data_), SPI_DATA_CHK_SIZE);

        dma_success_update = 1;
        dma_transmitting = 1;
        tick_count++;
        if (tick_count == tick_count_times)
        {
            tick_count = 0;
            LL_GPIO_TogglePin(GPIOA, LED_1_Pin | LED_2_Pin);
        }
    }
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)spi_data_, (uint8_t *)spi_cmd_, SPI_DATA_SIZE);
}

void init_motors_manager(void)
{
    motors_manager.error_check = Error_Check;
    motors_manager.receive_msgs = Receive_Msgs;
    motors_manager.send_msgs_id1 = Send_Msgs_ID1;
    motors_manager.send_msgs_id2 = Send_Msgs_ID2;
    motors_manager.send_msgs_id3 = Send_Msgs_ID3;
    motors_manager.pause_check = Pause_Check;

    init_can_manager();
    local_can_manager = get_can_manager_handle();
    local_can_manager->fdcan_stb();

    status_bits |= (0x7 << DISABLE_SHIFT);
    // NOTE: memset only for test
    // for (int i = 0; i < 6; i++) {
    //  memset(motors_manager.motor_cmds_.spi_cmd.motor_cmd[i].buffer, i + 1, 8);
    //}

    // send disable messgae after setup.
    disable_motors_once_setup();
}

// note: comment this in real robot
#ifdef UNPACK_DATA
void task_unpack_data(void)
{
    int32_t p_int = (motors_data.motor_data[Number_Monitor_Motor].buffer[0] << 8) |
                    motors_data.motor_data[Number_Monitor_Motor].buffer[1];
    int32_t v_int = (motors_data.motor_data[Number_Monitor_Motor].buffer[2] << 4) |
                    (motors_data.motor_data[Number_Monitor_Motor].buffer[3] >> 4);
    int32_t i_int = ((motors_data.motor_data[Number_Monitor_Motor].buffer[3] & 0xF) << 8) |
                    motors_data.motor_data[Number_Monitor_Motor].buffer[4];
    int32_t uq_int = (motors_data.motor_data[Number_Monitor_Motor].buffer[5] << 4) |
                     (motors_data.motor_data[Number_Monitor_Motor].buffer[6] >> 4);
    int32_t ud_int = ((motors_data.motor_data[Number_Monitor_Motor].buffer[6] & 0xF) << 8) |
                     motors_data.motor_data[Number_Monitor_Motor].buffer[7];
    int32_t acc_int = ((motors_data.motor_data[Number_Monitor_Motor].buffer[8]) << 8) |
                      motors_data.motor_data[Number_Monitor_Motor].buffer[9];
    int32_t coil_temp_int = ((motors_data.motor_data[Number_Monitor_Motor].buffer[10]) << 4) |
                            (motors_data.motor_data[Number_Monitor_Motor].buffer[11] >> 4);
    motor_data_monitor.work_bit_ = motors_data.motor_data[Number_Monitor_Motor].buffer[11] & 0xF;

    motor_data_monitor.p_data_ = uint_to_float(p_int, P_MIN, P_MAX, 16);
    motor_data_monitor.v_data_ = uint_to_float(v_int, V_MIN, V_MAX, 12);
    motor_data_monitor.tor_data_ = uint_to_float(i_int, KI_MIN, KI_MAX, 12);
    motor_data_monitor.uq_ = uint_to_float(uq_int, UQ_MIN, UQ_MAX, 12);
    motor_data_monitor.ud_ = uint_to_float(ud_int, UQ_MIN, UD_MAX, 12);
    motor_data_monitor.acc_ = uint_to_float(acc_int, ACC_MIN, ACC_MAX, 16);
    motor_data_monitor.coil_temp_ = uint_to_float(coil_temp_int, TEMP_MIN, TEMP_MAX, 12);
}
#endif

#ifdef fake_data
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float pos, vel, tau, uq, ud;
uint8_t fake_data_buf[8];
void trick_data(void)
{
    static uint32_t iter;
    iter++;
    pos = 2 * sinf((float)iter * 0.00001f);
    vel = 3 * sinf((float)iter * 0.00002f);
    tau = 10 * cosf((float)iter * 0.00003f);

    int p_int = float_to_uint(pos, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(vel, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(tau, KI_MIN, KI_MAX, 12);
    // TODO check the filter the uq ud data.

    int uq_int = float_to_uint(uq, UQ_MIN, UQ_MAX, 12);
    int ud_int = float_to_uint(ud, UD_MIN, UD_MAX, 12);

    fake_data_buf[0] = p_int >> 8;
    fake_data_buf[1] = p_int & 0xFF;
    fake_data_buf[2] = v_int >> 4;
    fake_data_buf[3] = ((v_int & 0xF) << 4) + (t_int >> 8);
    fake_data_buf[4] = t_int & 0xFF;
    fake_data_buf[5] = uq_int >> 4;
    fake_data_buf[6] = ((uq_int & 0xF) << 4) | ud_int >> 8;
    fake_data_buf[7] = ud_int & 0xFF;

    for (int i = 0; i < 6; i++)
    {
        memcpy(motors_data.motor_data[i].buffer, fake_data_buf, 8);
    }
}
#endif

// NOTE normally 0.48us.
void manager_task(void)
{
    // LL_GPIO_SetOutputPin(GPIOA, LED_1_Pin | LED_2_Pin);
    static uint32_t cali_flg;
    static uint32_t zero_flg;
    if (dma_success_update)
    {
        uint32_t ctrl_flg = motors_cmd.flag_cmd[0];

        cali_flg = (ctrl_flg & (0x1F << CMD_CALI_SHIFT));
        zero_flg = (ctrl_flg & (0x1F << CMD_ZERO_SHIFT));
        // if enable cmd and motors disable
        if ((ctrl_flg & 0x01) && (((status_bits >> DISABLE_SHIFT) & 0x7) == 0x7))
        {
            CONTROL_MODE_T control_mode = ((ctrl_flg >> 2) & 0x07);
            motors_manager.control_mode_ = control_mode;
            status_bits &= ~(0x7 << DISABLE_SHIFT);
            // if disable cmd and motors enable
        }
        else if (((status_bits & 0x7) == 0x7) && (ctrl_flg & (0x01 << 1)))
        {
            motors_manager.control_mode_ = disable;
            status_bits &= ~(0x7 << ENABLE_SHIFT);
            status_bits &= ~(0x1 << CTRL_SHIFT);
        }
        else if ((((status_bits >> ZERO_SHIFT) & 0x7) == 0) && (zero_flg != 0))
        {
            motors_manager.control_mode_ = zero;
            local_can_manager->zero_motor_id_ = zero_flg >> CMD_ZERO_SHIFT;
            status_bits |= (0x7 << ZERO_SHIFT);
        }
        else if ((((status_bits >> CALI_SHIFT) & 0x7) == 0) && (cali_flg != 0))
        {
            motors_manager.control_mode_ = cali;
            local_can_manager->cali_motor_id_ = cali_flg >> CMD_CALI_SHIFT;
            status_bits |= (0x7 << CALI_SHIFT);
        }
    }

    // all motor enable
    uint8_t ctrl_bit = ((status_bits >> CTRL_SHIFT) & 0x1);
    if (((status_bits & 0x7) == 0x7) & dma_transmitting & !ctrl_bit)
    {
        delay_us(100);
        motors_manager.control_mode_ = normal_ctrl;
        status_bits |= (0x1 << CTRL_SHIFT);
    }

#ifdef UNPACK_DATA
    if (dma_success_update)
    {
        task_unpack_data();
    }
#endif

#ifdef fake_data
    trick_data();
#endif
    // LL_GPIO_ResetOutputPin(GPIOA, LED_1_Pin | LED_2_Pin);
    // delay_ms(1);
    //  take the second byte: disable bit
}
