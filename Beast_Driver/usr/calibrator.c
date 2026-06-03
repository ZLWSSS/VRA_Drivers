#include "calibrator.h"
#include "config.h"
#include "error_status.h"
#include "flash_writer.h"
#include "gpio.h"
#include "math.h"
#include "motor_config.h"
#include "stm32g4xx_ll_gpio.h"
#include "string.h"
#include "usr_delay.h"

Calibrator_Handler_t cali_handle_;
// FLASH_EraseInitTypeDef erase_t;
// global vector for calibrate;
float __attribute((section(".RAM1"))) error[N_Vec_Size];
float __attribute((section(".RAM1"))) error_f[N_Vec_Size];
float __attribute((section(".RAM1"))) error_b[N_Vec_Size];
float __attribute((section(".CCM_RAM1"))) error_filt[N_Vec_Size];
uint32_t __attribute((section(".RAM1"))) raw_f[N_Vec_Size];
uint32_t __attribute((section(".RAM1"))) raw_b[N_Vec_Size];

// this func test ok
static void phase_check(SVPWM_Handler_t *svpwm_handle, Encoders_Handle_t *encoders_handle)
{
    float theta_start = 0;
    float theta_end = 0;
    float theta_ref = 0;
    theta_s_c_t theta_s_c_;
    theta_s_c_.sin_ = sinf(theta_ref);
    theta_s_c_.cos_ = cosf(theta_ref);
    float theta_actual = 0;
    qd_f_t V;
    V.d = V_cal;
    V.q = 0;
    int sample_counter = 0;
    // get abc pwm cnt
    svpwm_handle->pfct_svm(&V, &theta_s_c_, V_bus_cal);
    for (int i = 0; i < 20000; i++)
    {
        svpwm_handle->pfct_set_occupation();
        delay_us(200);
    }
    // check phase order
    while (theta_ref < 4 * M_PI)
    {
        theta_s_c_.sin_ = sinf(theta_ref);
        theta_s_c_.cos_ = cosf(theta_ref);
        // running time: 1.88us
        svpwm_handle->pfct_svm(&V, &theta_s_c_, V_bus_cal);
        svpwm_handle->pfct_set_occupation();
        theta_actual = encoders_handle->pfct_get_rotor_pos(cali_handle_.offset_lut);
        if (theta_ref == 0)
            theta_start = theta_actual;
        theta_ref += 0.001f;
        delay_us(100);
    }
    theta_end = encoders_handle->pfct_get_rotor_pos(cali_handle_.offset_lut);
    if (theta_end - theta_start > 0)
    {
        svpwm_handle->phase_order = 1;
    }
    else
    {
        svpwm_handle->phase_order = 0;
    }
}

// use malloc to save the sram, but nor work in this ide.
// TODO add anticogging calibration

// uint8_t error_indi[10];
float theta_ref = 0;
float theta_actual = 0;
float mean = 0;
int32_t index;
int raw_offset;
void calibrate(SVPWM_Handler_t *svpwm_handle, Encoders_Handle_t *encoders_handle)
{
    theta_ref = 0;
    // error_indi[0] = 1;

    theta_s_c_t theta_s_c_;
    theta_s_c_.sin_ = sinf(theta_ref);
    theta_s_c_.cos_ = cosf(theta_ref);
    qd_f_t V;
    V.d = V_cal;
    V.q = 0;
    //// 如果相序不对，直接返回；
    if (!svpwm_handle->phase_order)
    {
        Add_Error(Phase_Incorrect);
        return;
    }
    //  prepare for cali
    svpwm_handle->pfct_svm(&V, &theta_s_c_, V_bus_cal);
    for (int i = 0; i < 40000; i++)
    {
        svpwm_handle->pfct_set_occupation();
        delay_us(100);
    }
    // error_indi[1] = 1;
    //  foward
    for (int i = 0; i < N_Vec_Size; i++)
    {
        for (int j = 0; j < N_Increment_Sample; j++)
        {
            theta_ref += Delta_Angle;
            theta_s_c_.sin_ = sinf(theta_ref);
            theta_s_c_.cos_ = cosf(theta_ref);
            svpwm_handle->pfct_svm(&V, &theta_s_c_, V_bus_cal);
            svpwm_handle->pfct_set_occupation();
            delay_us(100);
        }
        theta_actual = encoders_handle->pfct_get_rotor_pos(cali_handle_.offset_lut);
        error_f[i] = theta_ref / (float)Npp - theta_actual;
        raw_f[i] = encoders_handle->pfct_get_rotor_encoder_raw_Data();
    }
    // error_indi[2] = 1;
    //  back
    for (int i = 0; i < N_Vec_Size; i++)
    {
        for (int j = 0; j < N_Increment_Sample; j++)
        {
            theta_ref -= Delta_Angle;
            theta_s_c_.sin_ = sinf(theta_ref);
            theta_s_c_.cos_ = cosf(theta_ref);
            svpwm_handle->pfct_svm(&V, &theta_s_c_, V_bus_cal);
            svpwm_handle->pfct_set_occupation();
            delay_us(100);
        }
        theta_actual = encoders_handle->pfct_get_rotor_pos(cali_handle_.offset_lut);
        error_b[i] = theta_ref / (float)Npp - theta_actual;
        raw_b[i] = encoders_handle->pfct_get_rotor_encoder_raw_Data();
    }

    // error_indi[3] = 1;
    //  eleoffset, take eleangle start from zero, record the mech angle, then record the ele angle
    float offset = 0;
    for (int i = 0; i < N_Vec_Size; i++)
    {
        offset += ((error_f[i] + error_b[N_Vec_Size - 1 - i]) / (2.f * N_Vec_Size));
    }
    offset = fmodf(offset * Npp, 2 * M_PI);

    // error_indi[4] = 1;

    encoders_handle->pfct_set_offset_ele(offset);

    for (int i = 0; i < N_Vec_Size; i++)
    {
        error[i] = 0.5f * (error_f[i] + error_b[N_Vec_Size - i - 1]);
    }
    for (int i = 0; i < N_Vec_Size; i++)
    {
        for (int j = 0; j < N_Offset_Section; j++)
        {
            int ind = (-N_Offset_Section / 2) + j + i; // Indexes from -window/2 to + window/2
            if (ind < 0)
            {
                ind += N_Vec_Size;
            }
            else if (ind > (N_Vec_Size - 1))
            {
                ind -= N_Vec_Size;
            }
            error_filt[i] += (error[ind] / (float)N_Offset_Section);
        }
        mean += (error_filt[i] / N_Vec_Size);
    }
    // take one group
    // 128 sector as a group
    raw_offset = (raw_f[0] + raw_b[N_Vec_Size - 1]) / 2;
    for (int i = 0; i < N_Offset_Section; i++)
    {
        index = (raw_offset >> encoder_cali_shift) + i;
        if (index > (N_Offset_Section - 1))
        {
            index -= N_Offset_Section;
        }
        cali_handle_.offset_lut[index] =
            (int)((error_filt[i * Npp] - mean) * (float)(MT6835_Resolution) / (2.0f * M_PI));
        delay_ms(1);
    }
    // cali done
    cali_handle_.flg_calibrated = 1;
    /* write data to flash
     * offset_lut float 4 bytes
     */
    flash_erase_address(1, 1); // erase bank2 page 1
    // program 4 bytes per time.
    // RCT6 only 256kb flash, sector 6 and sector 7 are useless
    // 128 * 4
    if (0 == flash_write_single_address(ADDR_FLASH_SECTOR_1, (uint64_t *)cali_handle_.offset_lut, N_Offset_Section_2))
    {
    }
    else
    {
        Add_Error(FLASH_Write_Error);
    }
    // eleoffset
    if (0 == flash_write_single_address((ADDR_FLASH_SECTOR_1 + N_Offset_Section_2 * 64), (uint64_t *)&offset, 1))
    {
    }
    else
    {
        Add_Error(FLASH_Write_Error);
    }
    // phase order
    if (0 == flash_write_single_address((ADDR_FLASH_SECTOR_1 + (N_Offset_Section_2 + 1) * 64),
                                        (uint64_t *)&(svpwm_handle->phase_order), 1))
    {
    }
    else
    {
        Add_Error(FLASH_Write_Error);
    }
}

void Calibrator_Init(void)
{
    cali_handle_.pfct_phase_check = phase_check;
    flash_read(ADDR_FLASH_SECTOR_1, (uint32_t *)cali_handle_.offset_lut, N_Offset_Section);
    // flash_read((ADDR_FLASH_SECTOR_4 + 4), (uint32_t *)p_handle->anticogging, ANTI_COGGING_SIZE);
    if ((cali_handle_.offset_lut[0] == -1) && (cali_handle_.offset_lut[N_Offset_Section - 1] == -1))
    {
        Add_Error(Cali_Error); // not cali yet
        memset(cali_handle_.offset_lut, 0, sizeof(int) * N_Offset_Section);
        cali_handle_.flg_calibrated = 0;
    }
    else
    {
        cali_handle_.flg_calibrated = 1;
    }
}

Calibrator_Handler_t *get_cali_handler(void)
{
    return &cali_handle_;
}
