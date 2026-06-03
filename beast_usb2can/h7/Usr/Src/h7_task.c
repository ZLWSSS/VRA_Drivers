#include "h7_task.h"
#include "stm32h7xx_ll_gpio.h"
#include "string.h"

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

uint8_t receive_flag = 0;

// attribute spi buffer to ram4
H7_Board_TypeDef h7_board;

extern USBD_HandleTypeDef hUsbDeviceHS;
volatile uint8_t spi_cmd_lock_spi3 = 0;
volatile uint8_t spi_cmd_lock_spi1 = 0;
volatile uint8_t spi_cmd_lock_spi4 = 0;
volatile uint8_t spi_data_lock_spi3 = 0;
volatile uint8_t spi_data_lock_spi1 = 0;
volatile uint8_t spi_data_lock_spi4 = 0;

// NOTE consume about 4.8us.
static uint8_t Receive_Usb_Cmd(uint8_t *usb_out_cmd_buf)
{
    // waiting for dma
    // LL_GPIO_SetOutputPin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin);
    while (spi_cmd_lock_spi1 | spi_cmd_lock_spi3 | spi_cmd_lock_spi4)
    {
    }

    spi_cmd_lock_spi3 = spi_cmd_lock_spi1 = spi_cmd_lock_spi4 = 1;

    USB_CMD_U *p_temp_usb_cmd_u = &h7_board.h7_usb->usb_cmd_u;
    memcpy(p_temp_usb_cmd_u->usb_cmd_pack_buf, usb_out_cmd_buf, USB_CMD_SIZE);
    uint32_t usb_cmd_checksum = data_checksum((uint32_t *)(&(p_temp_usb_cmd_u->usb_cmd_pack)), (USB_CMD_SIZE / 4) - 1);
    if (usb_cmd_checksum == p_temp_usb_cmd_u->usb_cmd_pack.checksum)
    {
        // check ok, convert usb cmd to motor_cmd format
        for (int i = 0; i < CHIP_NUMBERS; i++)
        {
            USB_CMD_TypeDef *usb_cmd = &h7_board.h7_usb->usb_cmd_u.usb_cmd_pack.usb_cmd[i];
            SPI_PROTOCAL_TypeDef *spi_cmd = h7_board.g474_chips[i];
            for (int j = 0; j < CHIP_CTRL_MOTOR_NUMBERS; j++)
            {
                int p_int = float_to_uint(usb_cmd->cmd_pack[j].p_cmd, P_MIN, P_MAX, 16);
                int v_int = float_to_uint(usb_cmd->cmd_pack[j].v_cmd, V_MIN, V_MAX, 12);
                int kp_int = float_to_uint(usb_cmd->cmd_pack[j].kp, KP_MIN, KP_MAX, 12);
                int kd_int = float_to_uint(usb_cmd->cmd_pack[j].kd, KD_MIN, KD_MAX, 12);
                int t_int = float_to_uint(usb_cmd->cmd_pack[j].t_ff, KI_MIN, KI_MAX, 12);
                spi_cmd->spi_cmd_u.spi_cmd.motor_cmd[j].buffer[0] = p_int >> 8;
                spi_cmd->spi_cmd_u.spi_cmd.motor_cmd[j].buffer[1] = p_int & 0xFF;
                spi_cmd->spi_cmd_u.spi_cmd.motor_cmd[j].buffer[2] = v_int >> 4;
                spi_cmd->spi_cmd_u.spi_cmd.motor_cmd[j].buffer[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
                spi_cmd->spi_cmd_u.spi_cmd.motor_cmd[j].buffer[4] = kp_int & 0xFF;
                spi_cmd->spi_cmd_u.spi_cmd.motor_cmd[j].buffer[5] = kd_int >> 4;
                spi_cmd->spi_cmd_u.spi_cmd.motor_cmd[j].buffer[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
                spi_cmd->spi_cmd_u.spi_cmd.motor_cmd[j].buffer[7] = t_int & 0xff;
            }
            spi_cmd->spi_cmd_u.spi_cmd.flag_cmd[0] = usb_cmd->chip_flag[0];
        }
        spi_cmd_lock_spi3 = spi_cmd_lock_spi1 = spi_cmd_lock_spi4 = 0;
        // LL_GPIO_ResetOutputPin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin);
        return 1;
    }
    else
    {
        spi_cmd_lock_spi3 = spi_cmd_lock_spi1 = spi_cmd_lock_spi4 = 0;
        return 0;
    }
}

// NOTE cost 8 us.

static void Pack_Usb_Data(void)
{
    // LL_GPIO_SetOutputPin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin);
    if (receive_flag == 0x07)
    {
        while (spi_data_lock_spi1 | spi_data_lock_spi3 | spi_data_lock_spi4)
        {
        }
        spi_data_lock_spi3 = spi_data_lock_spi1 = spi_data_lock_spi4 = 1;
        for (int i = 0; i < CHIP_NUMBERS; i++)
        {
            USB_DATA_TypeDef *usb_data = &h7_board.h7_usb->usb_data_u.usb_data_pack.usb_data[i];
            // usb_data = &h7_board.h7_usb->usb_data_u.usb_data_pack.usb_data[i];
            SPI_PROTOCAL_TypeDef *spi_data = h7_board.g474_chips[i];
            // spi_data = h7_board.g474_chips[i];
            for (int j = 0; j < CHIP_CTRL_MOTOR_NUMBERS; j++)
            {
                MOTOR_MESSAGE_DATA_T *p_data = &(spi_data->spi_data_u.spi_data.motor_data[j]);
                // p_data = spi_data->spi_data_u.spi_data.motor_data;

                int32_t p_int = (p_data->buffer[0] << 8) | p_data->buffer[1];
                int32_t v_int = (p_data->buffer[2] << 4) | (p_data->buffer[3] >> 4);
                int32_t i_int = ((p_data->buffer[3] & 0xF) << 8) | p_data->buffer[4];
                int32_t uq_int = (p_data->buffer[5] << 4) | (p_data->buffer[6] >> 4);
                int32_t ud_int = ((p_data->buffer[6] & 0xF) << 8) | p_data->buffer[7];
                int32_t acc_int = ((p_data->buffer[8]) << 8) | p_data->buffer[9];
                int32_t coil_temp_int = ((p_data->buffer[10]) << 4) | (p_data->buffer[11] >> 4);

                usb_data->data_pack[j].p_data = uint_to_float(p_int, P_MIN, P_MAX, 16);
                usb_data->data_pack[j].v_data = uint_to_float(v_int, V_MIN, V_MAX, 12);
                usb_data->data_pack[j].t_data = uint_to_float(i_int, KI_MIN, KI_MAX, 12);
                usb_data->data_pack[j].uq = uint_to_float(uq_int, UQ_MIN, UQ_MAX, 12);
                usb_data->data_pack[j].ud = uint_to_float(ud_int, UQ_MIN, UD_MAX, 12);
                usb_data->data_pack[j].acc = uint_to_float(acc_int, ACC_MIN, ACC_MAX, 16);
                usb_data->data_pack[j].temp_coil = uint_to_float(coil_temp_int, TEMP_MIN, TEMP_MAX, 12);
                h7_board.h7_usb->usb_data_u.usb_data_pack.motor_status_ |= (p_data->buffer[11] & 0xF) << (j + 6 * i);
            }
        }

        h7_board.h7_usb->usb_data_u.usb_data_pack.checksum =
            data_checksum((uint32_t *)(&(h7_board.h7_usb->usb_data_u.usb_data_pack)), USB_DATA_CHK_SIZE);
        receive_flag = 0;
        spi_data_lock_spi3 = spi_data_lock_spi1 = spi_data_lock_spi4 = 0;
    }
    // LL_GPIO_ResetOutputPin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin);
}

// NOTE each spi transmission cost about 25.4us
static void Send_Spi_Cmd_DMA(void)
{
    // lock cmd and data while dma transmitting
    // while (spi_cmd_lock_spi1 | spi_cmd_lock_spi3 | spi_cmd_lock_spi4) {
    //}
    // spi_cmd_lock_spi1 = spi_cmd_lock_spi3 = spi_cmd_lock_spi4 = 1;
    // while (spi_data_lock_spi1 | spi_data_lock_spi3 | spi_data_lock_spi4) {
    //}
    // spi_data_lock_spi3 = spi_data_lock_spi1 = spi_data_lock_spi4 = 1;

    if (h7_board.cmd_update)
    {
        for (int i = 0; i < CHIP_NUMBERS; i++)
        {
            h7_board.g474_chips[i]->spi_cmd_u.spi_cmd.checksum_tx_message =
                data_checksum((uint32_t *)(&(h7_board.g474_chips[i]->spi_cmd_u.spi_cmd)), SPI_CMD_CHK_SIZE);
        }
        h7_board.cmd_update = 0;
    }

    // LL_GPIO_SetOutputPin(PIN_TEST1_GPIO_Port, PIN_TEST1_Pin);
    for (int i = 0; i < CHIP_NUMBERS; i++)
    {
        LL_GPIO_ResetOutputPin(h7_board.g474_chips[i]->SPI_CS_Port, h7_board.g474_chips[i]->SPI_CS_Pin);
        // resize the driver.
        HAL_SPI_TransmitReceive_DMA(h7_board.g474_chips[i]->Spi_Channel,
                                    (h7_board.g474_chips[i]->spi_cmd_u.spi_cmd_buff),
                                    (h7_board.g474_chips[i]->spi_data_u.spi_data_buff), SPI_DATA_SIZE);
    }
}

uint32_t data_rx_checksize;
static uint8_t Check_Chips_Data(SPI_PROTOCAL_TypeDef *_spi_protocal)
{
    data_rx_checksize = data_checksum((uint32_t *)&(_spi_protocal->spi_data_u.spi_data), SPI_DATA_CHK_SIZE);
    // if ok, copy temp to rx_data
    if (data_rx_checksize == _spi_protocal->spi_data_u.spi_data.checksum_rx_message)
    {
        // checksum ok，
        return 1;
    }
    else
    {
        return 0;
    }
}

static uint8_t Send_Usb_Data(void)
{
    uint8_t usb_status =
        USBD_CUSTOM_HID_SendReport(&hUsbDeviceHS, h7_board.h7_usb->usb_data_u.usb_data_pack_buf, USB_DATA_SIZE);

    return 1;
}

// NOTE cost 8.6 us.
void send_data(void)
{
    // LL_GPIO_SetOutputPin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin);
    if (h7_board.run)
    {
        h7_board.pack_usb_data();
        h7_board.send_usb_data();
    }
    // h7_board.receive_usb_cmd(h7_board.h7_usb->usb_cmd_u.usb_cmd_pack_buf);
    // LL_GPIO_ResetOutputPin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin);
}

void init_h7_board(void)
{
    init_spi_protocals();
    set_h7board_usb();
    usb_handle_init();

    h7_board.h7_usb = get_usb_handle();
    for (uint8_t i = 0; i < CHIP_NUMBERS; i++)
    {
        h7_board.g474_chips[i] = get_spi_protocal(i);
    }

    h7_board.check_chips_data = Check_Chips_Data;
    h7_board.receive_usb_cmd = Receive_Usb_Cmd;
    h7_board.send_spi_cmd_dma = Send_Spi_Cmd_DMA;
    h7_board.send_usb_data = Send_Usb_Data;
    h7_board.pack_usb_data = Pack_Usb_Data;
    h7_board.run = 0;
    h7_board.hit_miss = 10;
    // init local temp ptr
}

H7_Board_TypeDef *get_h7_board(void)
{
    return &h7_board;
}

// ticker isr: 6.5us
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // LL_GPIO_SetOutputPin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin);
    // LL_GPIO_SetOutputPin(PIN_TEST1_GPIO_Port, PIN_TEST1_Pin);
    if (h7_board.run)
    {
        h7_board.send_spi_cmd_dma();
        h7_board.hit_miss++;
    }
}

// NOTE first channel: 25.6us second: 28.3us third: 30.3us; jittering due to cmd checksum. this value is the maxiumum.
uint32_t check_[3];
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3)
    {
        // uint8_t check = ;
        if (h7_board.check_chips_data(h7_board.g474_chips[0]))
        {
            receive_flag |= 0x01;
            check_[0]++;
            LL_GPIO_SetOutputPin(h7_board.g474_chips[0]->SPI_CS_Port, h7_board.g474_chips[0]->SPI_CS_Pin);
            // spi_cmd_lock_spi3 = spi_data_lock_spi3 = 0;
            // LL_GPIO_ResetOutputPin(PIN_TEST1_GPIO_Port, PIN_TEST1_Pin);
        }
    }
    else if (hspi == &hspi1)
    {
        if (h7_board.check_chips_data(h7_board.g474_chips[1]))
        {
            check_[1]++;
            receive_flag |= 0x01 << 1;
            LL_GPIO_SetOutputPin(h7_board.g474_chips[1]->SPI_CS_Port, h7_board.g474_chips[1]->SPI_CS_Pin);

            // spi_cmd_lock_spi1 = spi_data_lock_spi1 = 0;
        }
    }
    else if (hspi == &hspi4)
    {
        if (h7_board.check_chips_data(h7_board.g474_chips[2]))
        {
            check_[2]++;
            receive_flag |= 0x01 << 2;
            LL_GPIO_SetOutputPin(h7_board.g474_chips[2]->SPI_CS_Port, h7_board.g474_chips[2]->SPI_CS_Pin);
            // LL_GPIO_ResetOutputPin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin);
            //  spi_cmd_lock_spi4 = spi_data_lock_spi4 = 0;
        }
    }
}