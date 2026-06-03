#include "csine_cordic.h"
#include "math.h"
#include "config.h"

cordic_t cordic_;

static int Value_To_CORDIC31(float Value, float Coefficient)
{
    int CORDIC31;
    CORDIC31 = (int)((Value / Coefficient) * 0x80000000);
    return CORDIC31;
}

static void CORDIC31_To_Value(int CORDIC31, float *RES)
{
    if (CORDIC31 & 0x80000000)
    { /*为负数*/
        CORDIC31 = CORDIC31 & 0x7FFFFFFF;
        *RES = (((float)(CORDIC31)-0x80000000) / 0x80000000);
    }
    else
    { /*为正数*/
        *RES = (float)(CORDIC31) / 0x80000000;
    }
}

static void Get_Cordic_Result(float *etheta)
{
    cordic_.cordic_etheta_ = Value_To_CORDIC31(*etheta, M_PI);
    LL_CORDIC_WriteData(CORDIC, cordic_.cordic_etheta_);
    cordic_.cordic_cos_ = LL_CORDIC_ReadData(CORDIC);
    cordic_.cordic_sin_ = LL_CORDIC_ReadData(CORDIC);
    CORDIC31_To_Value(cordic_.cordic_cos_, &cordic_.cordic_output_.cos_);
    CORDIC31_To_Value(cordic_.cordic_sin_, &cordic_.cordic_output_.sin_);
}

void Cordic_Handle_Init(void)
{
    LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_COSINE, LL_CORDIC_PRECISION_6CYCLES, LL_CORDIC_SCALE_0,
                     LL_CORDIC_NBWRITE_1, LL_CORDIC_NBREAD_2, LL_CORDIC_INSIZE_32BITS, LL_CORDIC_OUTSIZE_32BITS);
    cordic_.pfct_get_csine = Get_Cordic_Result;
}

cordic_t *get_cordic_handle(void)
{
    return &cordic_;
}
