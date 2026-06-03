#ifndef ERROR_HELPER_H_
#define ERROR_HELPER_H_

#include "stdint.h"

#define N_MAX_ERROR 10U

typedef enum Error_Enum
{
    NO_Error = 0,
    DRV_Init_Error,
    FLASH_Write_Error, // flash write error, check voltage
    FLASH_Read_Error,
    Cali_Error,         // not calibrate when starting
    Zero_Positon_Error, // not zero
    Phase_Incorrect,
    DRV_Fault_OCP,
    DRV_Fault_GDF,
    DRV_Fault_UVLO,
    DRV_Fault_OTSD,
    DRV_Fault_VDS_HA,
    DRV_Fault_VDS_LA,
    DRV_Fault_VDS_HB,
    DRV_Fault_VDS_LB,
    DRV_Fault_VDS_HC,
    DRV_Fault_VDS_LC,
    DRV_Fault_SA_OC,
    DRV_Fault_SB_OC,
    DRV_Fault_SC_OC,
    DRV_Fault_OTW,
    DRV_Fault_CPUV,
    DRV_Fault_VGS_HA,
    DRV_Fault_VGS_LA,
    DRV_Fault_VGS_HB,
    DRV_Fault_VGS_LB,
    DRV_Fault_VGS_HC,
    DRV_Fault_VGS_LC,
    ADC1_CALIB_ERROR,
    ADC2_CALIB_ERROR,
    ADC3_CALIB_ERROR,
    ADC5_CALIB_ERROR,
    DRV_PROTECT_ERROR,
} Error_Enum_t;

typedef struct Error_Type
{
    Error_Enum_t error_list[10];
    uint16_t error_index;
} Error_Type_t;

void Add_Error(Error_Enum_t error_state);
#endif