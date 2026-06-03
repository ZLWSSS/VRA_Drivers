#include "encoders.h"
#include "config.h"
#include "error_status.h"
#include "flash_writer.h"
#include "math.h"
#include "motor_config.h"
#include "string.h"

Encoders_Handle_t encoders_handle_;

// ATTENTION: Please note the flash writer address.

// constant parameters.
#define aver_vecsize 10U
#define cpr2rad_factor (2.0f * M_PI / (float)(MT6835_Resolution))
// help for calculating ****************************
int off_1, off_2, off_interp;
int mt6835_raw_data = 0;
volatile uint8_t first_run = 1;
float aver_velVec[aver_vecsize];
int32_t spi_buff_data[3];
// ***************************************************

// make sure which is the rotor encoder.
static void Spi_Read(void)
{
    if (!LL_SPI_IsEnabled(SPI2))
    {
        LL_SPI_Enable(SPI2);
    }
    // TX
    LL_GPIO_ResetOutputPin(ENCODER_CS_GPIO_Port, ENCODER_CS_Pin);
    while (!LL_SPI_IsActiveFlag_TXE(SPI2))
    {
    };
    LL_SPI_TransmitData16(SPI2, MT6835_angle);

    while (!LL_SPI_IsActiveFlag_RXNE(SPI2))
    {
    };
    spi_buff_data[0] = LL_SPI_ReceiveData16(SPI2);

    while (!LL_SPI_IsActiveFlag_TXE(SPI2))
    {
    };
    LL_SPI_TransmitData16(SPI2, MT6835_angle);

    while (!LL_SPI_IsActiveFlag_RXNE(SPI2))
    {
    };
    spi_buff_data[1] = LL_SPI_ReceiveData16(SPI2);

    while (!LL_SPI_IsActiveFlag_TXE(SPI2))
    {
    };
    LL_SPI_TransmitData16(SPI2, MT6835_nop);

    while (!LL_SPI_IsActiveFlag_RXNE(SPI2))
    {
    };
    spi_buff_data[2] = LL_SPI_ReceiveData16(SPI2);
    LL_GPIO_SetOutputPin(ENCODER_CS_GPIO_Port, ENCODER_CS_Pin);
}

static void set_offset_ele(float value)
{
    encoders_handle_.offset_ele = value;
}

static void foc_encoder_sample(int *lut_offset)
{
    Spi_Read();

    encoders_handle_.rotor_pos_cpr_last = encoders_handle_.rotor_pos_cpr;
    mt6835_raw_data = (spi_buff_data[1] << 5) | (spi_buff_data[2] >> 11);
    // interpolate, take 7 bits
    // upper 7 bit: sectors, lower 14 bit: precise
    // 0 0000 0000 0000 0000 0000
    off_1 = lut_offset[mt6835_raw_data >> 14];
    off_2 = lut_offset[((mt6835_raw_data >> 14) + 1) % 128];
    // only take 7 bit
    off_interp = off_1 + (((off_2 - off_1) * (mt6835_raw_data - ((mt6835_raw_data >> 14) << 14))) >> 14);
    // interpolate done
    encoders_handle_.rotor_pos_cpr = mt6835_raw_data + off_interp;

    if (!first_run)
    {
        if ((encoders_handle_.rotor_pos_cpr - encoders_handle_.rotor_pos_cpr_last) > Rotor_Encoder_Half_CPR)
        {
            encoders_handle_.loop_count -= 1;
        }
        else if ((encoders_handle_.rotor_pos_cpr - encoders_handle_.rotor_pos_cpr_last) < (-Rotor_Encoder_Half_CPR))
        {
            encoders_handle_.loop_count += 1;
        }
    }
    else
    {
        // Run only once
        float pos_init = (float)(encoders_handle_.rotor_pos_cpr * cpr2rad_factor);
        float zero = encoders_handle_.offset_me;
        float range;
        if (zero > M_PI)
        {
            range = zero - M_PI;
            if (pos_init < range)
                encoders_handle_.loop_count++;
        }
        else
        {
            range = zero + M_PI;
            if (pos_init > range)
                encoders_handle_.loop_count--;
        }
        first_run = 0;
    }
    // position
    encoders_handle_.rotor_pos_old_fl_ = encoders_handle_.rotor_pos_fl_;
    encoders_handle_.rotor_pos_fl_ = (float)(encoders_handle_.rotor_pos_cpr * cpr2rad_factor) +
                                     (encoders_handle_.loop_count * M_PI * 2.f) - encoders_handle_.offset_me;
    // average filter
    float vel = (encoders_handle_.rotor_pos_fl_ - encoders_handle_.rotor_pos_old_fl_) * Inverse_T_Pwm;
    vel = LPF_ALpha_Vel * vel + LPF_Beta_Vel * encoders_handle_.rotor_vel_;
    // test here: copy value: 2.1us
    // total 6.8us
    float sum = vel;
    for (int i = 1; i < (aver_vecsize); i++)
    {
        aver_velVec[aver_vecsize - i] = aver_velVec[aver_vecsize - i - 1];
        sum += aver_velVec[aver_vecsize - i];
    }
    aver_velVec[0] = vel;
    encoders_handle_.rotor_vel_ = sum / aver_vecsize;
    float ele_pos = (cpr2rad_factor * (float)((Npp * encoders_handle_.rotor_pos_cpr) % MT6835_Resolution)) +
                    encoders_handle_.offset_ele;
    if (ele_pos < (-M_PI))
        ele_pos += MI_PI2;
    else if (ele_pos > M_PI)
        ele_pos -= MI_PI2;
    encoders_handle_.pos_ele_ = ele_pos;
    encoders_handle_.vel_ele_ = Npp * encoders_handle_.rotor_vel_;
    encoders_handle_.pos_flange_ = encoders_handle_.rotor_pos_fl_ * Inverse_GR;
    encoders_handle_.flange_vel_last_ = encoders_handle_.flange_vel_;
    encoders_handle_.flange_vel_ = encoders_handle_.rotor_vel_ * Inverse_GR;
    encoders_handle_.flange_acc_ = (encoders_handle_.flange_vel_ - encoders_handle_.flange_vel_last_) * 40000.f;
}

static float get_rotor_pos(int *lut_offset)
{
    Spi_Read();
    encoders_handle_.rotor_pos_cpr_last = encoders_handle_.rotor_pos_cpr;
    mt6835_raw_data = (spi_buff_data[1] << 5) | (spi_buff_data[2] >> 11);
    // interpolate, take 7 bits
    // upper 7 bit: sectors, lower 14 bit: precise
    // 0 0000 0000 0000 0000 0000
    off_1 = lut_offset[mt6835_raw_data >> 14];
    off_2 = lut_offset[((mt6835_raw_data >> 14) + 1) % 128];
    // only take 7 bit
    off_interp = off_1 + (((off_2 - off_1) * (mt6835_raw_data - ((mt6835_raw_data >> 14) << 14))) >> 14);
    // interpolate done
    encoders_handle_.rotor_pos_cpr = mt6835_raw_data + off_interp;
    // interpolate done
    if ((encoders_handle_.rotor_pos_cpr - encoders_handle_.rotor_pos_cpr_last) > Rotor_Encoder_Half_CPR)
    {
        encoders_handle_.loop_count -= 1;
    }
    else if ((encoders_handle_.rotor_pos_cpr - encoders_handle_.rotor_pos_cpr_last) < (-Rotor_Encoder_Half_CPR))
    {
        encoders_handle_.loop_count += 1;
    }
    encoders_handle_.rotor_pos_old_fl_ = encoders_handle_.rotor_pos_fl_;
    encoders_handle_.rotor_pos_fl_ = (float)(encoders_handle_.rotor_pos_cpr * cpr2rad_factor) +
                                     (encoders_handle_.loop_count * M_PI * 2.f) - encoders_handle_.offset_me;
    return encoders_handle_.rotor_pos_fl_;
}

static uint32_t get_rotor_encoder_raw_data(void)
{
    Spi_Read();
    encoders_handle_.rotor_pos_cpr_last = encoders_handle_.rotor_pos_cpr;
    mt6835_raw_data = (spi_buff_data[1] << 5) | (spi_buff_data[2] >> 11);
    return (mt6835_raw_data);
}

static void set_zero(int *lut_offset)
{
    encoders_handle_.loop_count = 0;
    encoders_handle_.offset_me = 0;
    encoders_handle_.offset_me = encoders_handle_.pfct_get_rotor_pos(lut_offset);
    flash_erase_address(0, 1); // bank 2 page 0
    // program 4 bytes per time.
    // RCT6 only 256kb flash, sector 6 and sector 7 are useless
    if (0 == flash_write_single_address(ADDR_FLASH_SECTOR_0, (uint64_t *)&(encoders_handle_.offset_me), 1))
    {
    }
    else
    {
        Add_Error(FLASH_Write_Error);
    }
    //// 64-bit write length, so add 64
    // if (0 == flash_write_single_address(ADDR_FLASH_SECTOR_0 + 64, (uint64_t
    // *)&(encoders_handle->offset_flange), 1)) { } else {
    //   Add_Error(FLASH_Write_Error);
    // }
}

void Encoders_Handler_Init(void)
{
    memset(&encoders_handle_, 0, sizeof(Encoders_Handle_t));
    encoders_handle_.pfct_foc_sample = foc_encoder_sample;
    encoders_handle_.pfct_get_rotor_encoder_raw_Data = get_rotor_encoder_raw_data;
    encoders_handle_.pfct_get_rotor_pos = get_rotor_pos;
    encoders_handle_.pfct_set_offset_ele = set_offset_ele;
    encoders_handle_.pfct_set_zero = set_zero;

    flash_read(ADDR_FLASH_SECTOR_0, (uint32_t *)&(encoders_handle_.offset_me), 1);
    // 128 扇区/2
    flash_read((ADDR_FLASH_SECTOR_1 + N_Offset_Section_2 * 64), (uint32_t *)&(encoders_handle_.offset_ele), 1);
    if (isnan(encoders_handle_.offset_me))
    {
        encoders_handle_.flg_zero = 0;
        memset(&(encoders_handle_.offset_me), 0, sizeof(float));
        memset(&(encoders_handle_.offset_ele), 0, sizeof(float));
        Add_Error(Zero_Positon_Error);
    }
    else
    {
        // make sure the loop starts from zero.
        encoders_handle_.rotor_pos_cpr = encoders_handle_.rotor_pos_cpr_last = MT6835_Resolution / 2;
        mt6835_raw_data = MT6835_Resolution / 2;
        encoders_handle_.flg_zero = 1;
    }
}

Encoders_Handle_t *get_encoder_handler(void)
{
    return &encoders_handle_;
}