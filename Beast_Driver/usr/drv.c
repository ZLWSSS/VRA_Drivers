#include "drv.h"
#include "error_status.h"
#include "spi.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_spi.h"
#include "usr_delay.h"

// LED Status:
// Green Light: Enable;
// Blue Light: Function Indicator;
// Red Light: Error State;

// ********************** global variables ******************************
uint16_t cmd_in;
uint16_t cmd_csa, cmd_dcr, cmd_ocpc, check_csa, check_dcr, check_ocpc;
uint16_t cmd_gdhs, cmd_gdls, check_gdhs, check_gdls;
uint16_t fsr1;
uint16_t fsr2;
// ***********************************************************************

Drv_Handler_t drv_handle_;

static uint16_t drv_write(uint16_t cmd)
{
    cmd_in = cmd;
    uint16_t readout;
    if (!LL_SPI_IsEnabled(SPI1))
    {
        LL_SPI_Enable(SPI1);
    }
    LL_GPIO_ResetOutputPin(DRV_CS_GPIO_Port, DRV_CS_Pin);
    delay_us(2);
    while (!LL_SPI_IsActiveFlag_TXE(SPI1))
    {
    };
    LL_SPI_TransmitData16(SPI1, cmd);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
    {
    };
    readout = LL_SPI_ReceiveData16(SPI1);
    LL_GPIO_SetOutputPin(DRV_CS_GPIO_Port, DRV_CS_Pin);
    delay_us(2);
    return readout;
}

// read ok
uint16_t drv_read(uint16_t reg)
{
    uint16_t cmd = ((1 << 15) | reg << 11);
    uint16_t read;
    if (!LL_SPI_IsEnabled(SPI1))
    {
        LL_SPI_Enable(SPI1);
    }
    LL_GPIO_ResetOutputPin(DRV_CS_GPIO_Port, DRV_CS_Pin);
    delay_us(2);
    while (!LL_SPI_IsActiveFlag_TXE(SPI1))
    {
    };
    LL_SPI_TransmitData16(SPI1, cmd);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
    {
    };
    read = LL_SPI_ReceiveData16(SPI1);
    LL_GPIO_SetOutputPin(DRV_CS_GPIO_Port, DRV_CS_Pin);
    delay_us(2);
    // LL_GPIO_ResetOutputPin(DRV_CS_GPIO_Port, DRV_CS_Pin);
    // delay_us(50);
    // HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&cmd, (uint8_t *)&read, 1, 100);
    // delay_us(50);
    // LL_GPIO_SetOutputPin(DRV_CS_GPIO_Port, DRV_CS_Pin);
    return read;
}

// drv test ok.
// TODO Check the setup
void drv_setup(void)
{
    // let drv work
    drv_handle_.pfct_drv_work();
    delay_us(100);
    drv_handle_.pfct_drv_calibrate();
    delay_ms(100);
    cmd_csa = (reg_csac << 11) | (1 << 9) | (CSA_GAIN_20 << 6) | SEN_LVL_0_25;
    drv_write(cmd_csa);
    delay_ms(100);
    cmd_ocpc = (reg_ocpc << 11) | (TRETRY_4MS << 10) | (DEADTIME_400NS << 8) | (OCP_RETRY << 6) | (OCP_DEG_8US << 4) |
               (VDS_LVL_0_7);
    drv_write(cmd_ocpc);

    //cmd_csa = (reg_csac << 11) | (1 << 9) | (CSA_GAIN_20 << 6) | SEN_LVL_1_0;
    //drv_write(cmd_csa);
    //delay_ms(100);
    //cmd_ocpc = (reg_ocpc << 11) | (TRETRY_4MS << 10) | (DEADTIME_400NS << 8) | (OCP_RETRY << 6) | (OCP_DEG_8US << 4) |
    //           (VDS_LVL_1_0);
    //drv_write(cmd_ocpc);
    //delay_ms(100);

    cmd_dcr = (reg_dc << 11) | (PWM_MODE_3X << 5) | (0x01);
    drv_write(cmd_dcr);
    delay_ms(100);

    cmd_gdls = (reg_gdl << 11) | (TDRIVE_500NS) << 8 | (IDRIVEP_LS_700MA) << 4 | (IDRIVEN_LS_800MA);
    drv_write(cmd_gdls);
    delay_ms(100);

    cmd_gdhs = (reg_gdh << 11) | (IDRIVEP_HS_1000MA) << 4 | (IDRIVEN_HS_1100MA);
    drv_write(cmd_gdhs);
    delay_ms(100);
}

// Drv Enable:
void drv_enable(void)
{
    uint16_t cmd = (drv_read(reg_dc)) & (~(0x01 << 2));
    cmd = ((reg_dc << 11) | cmd);
    drv_write(cmd);
    LL_GPIO_ResetOutputPin(LED_G_GPIO_Port, LED_G_Pin);
}

void drv_disble(void) // put MOSFET in Hi-Z status
{
    uint16_t cmd = (drv_read(reg_dc) | (1 << 2));
    cmd = ((reg_dc << 11) | cmd);
    drv_write(cmd);
    LL_GPIO_SetOutputPin(LED_G_GPIO_Port, LED_G_Pin);
}

static void drv_check_fault(void)
{
    fsr1 = drv_read(reg_fs1);
    delay_us(10);
    fsr2 = drv_read(reg_vs2);
    delay_us(10);
    if (fsr1 & (1 << 10))
    {
    }
    if (fsr1 & (1 << 9))
    {
        Add_Error(DRV_Fault_OCP);
    }
    if (fsr1 & (1 << 8))
    {
        Add_Error(DRV_Fault_GDF);
    }
    if (fsr1 & (1 << 7))
    {
        Add_Error(DRV_Fault_UVLO);
    }
    if (fsr1 & (1 << 6))
    {
        Add_Error(DRV_Fault_OTSD);
    }
    if (fsr1 & (1 << 5))
    {
        Add_Error(DRV_Fault_VDS_HA);
    }
    if (fsr1 & (1 << 4))
    {
        Add_Error(DRV_Fault_VDS_LA);
    }
    if (fsr1 & (1 << 3))
    {
        Add_Error(DRV_Fault_VDS_HB);
    }
    if (fsr1 & (1 << 2))
    {
        Add_Error(DRV_Fault_VDS_LB);
    }
    if (fsr1 & (1 << 1))
    {
        Add_Error(DRV_Fault_VDS_HC);
    }
    if (fsr1 & (1))
    {
        Add_Error(DRV_Fault_VDS_LC);
    }

    if (fsr2 & (1 << 10))
    {
        Add_Error(DRV_Fault_SA_OC);
    }
    if (fsr2 & (1 << 9))
    {
        Add_Error(DRV_Fault_SB_OC);
    }
    if (fsr2 & (1 << 8))
    {
        Add_Error(DRV_Fault_SC_OC);
    }
    if (fsr2 & (1 << 7))
    {
        Add_Error(DRV_Fault_OTW);
    }
    if (fsr2 & (1 << 6))
    {
        Add_Error(DRV_Fault_CPUV);
    }
    if (fsr2 & (1 << 5))
    {
        Add_Error(DRV_Fault_VGS_HA);
    }
    if (fsr2 & (1 << 4))
    {
        Add_Error(DRV_Fault_VGS_LA);
    }
    if (fsr2 & (1 << 3))
    {
        Add_Error(DRV_Fault_VGS_HB);
    }
    if (fsr2 & (1 << 2))
    {
        Add_Error(DRV_Fault_VGS_LB);
    }
    if (fsr2 & (1 << 1))
    {
        Add_Error(DRV_Fault_VGS_HC);
    }
    if (fsr2 & (1))
    {
        Add_Error(DRV_Fault_VGS_LC);
    }
}

static void drv_work(void) // set enable pin, do not use this func
{
    LL_GPIO_SetOutputPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin);
    delay_us(1);
    LL_GPIO_SetOutputPin(DRV_INL_GPIO_Port, DRV_INL_Pin);
}

static void drv_sleep(void) // set drv in sleep mode, do not use this fun
{
    LL_GPIO_ResetOutputPin(DRV_INL_GPIO_Port, DRV_INL_Pin);
    delay_us(1);
    LL_GPIO_ResetOutputPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin);
}

static void drv_calibrate(void)
{
    uint16_t cmd = (1 << 4) | (1 << 3) | (1 << 2);
    drv_write((reg_csac << 11) | cmd);
}

void Drv_Init(void)
{
    drv_handle_.pfct_drv_calibrate = drv_calibrate;
    drv_handle_.pfct_drv_disable = drv_disble;
    drv_handle_.pfct_drv_enable = drv_enable;
    drv_handle_.pfct_drv_setup = drv_setup;
    drv_handle_.pfct_drv_sleep = drv_sleep;
    drv_handle_.pfct_drv_work = drv_work;
    drv_handle_.pfct_drv_check = drv_check_fault;
    drv_handle_.pfct_drv_setup();
    delay_ms(100);
    check_ocpc = drv_read(reg_ocpc);
    delay_ms(100);
    check_csa = drv_read(reg_csac);
    delay_ms(100);
    check_dcr = drv_read(reg_dc);
    cmd_dcr = (cmd_dcr & ~(0x1));

    delay_ms(100);
    check_gdhs = drv_read(reg_gdh);
    delay_ms(100);
    check_gdls = drv_read(reg_gdl);
    if (((cmd_csa & 0x7FF) == check_csa) && ((cmd_dcr & 0x7FF) == check_dcr) && ((cmd_ocpc & 0x7FF) == check_ocpc) &&
        ((cmd_gdls & 0x3FF) == (check_gdls & 0x3FF)) && ((cmd_gdhs & 0xFF) == (check_gdhs & 0xFF)))
    {
        drv_handle_.DRV_Status = 1;
    }
    else
    {
        drv_handle_.DRV_Status = 0;
        Add_Error(DRV_Init_Error);
        drv_check_fault();
    }
    drv_handle_.pfct_drv_disable();
}

Drv_Handler_t *get_drv_handler(void)
{
    return &drv_handle_;
}