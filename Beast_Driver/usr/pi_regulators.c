#include "pi_regulators.h"
#include "config.h"
#include "math.h"
#include "motor_config.h"

Cur_PI_Handler_t cur_pi_regulator_;
Vel_PI_Handler_t vel_pi_regulator_;

// ************************************* self min-max function ************************************
static float my_fmax(float x, float y)
{
    return (x > y) ? x : y;
}

static float my_fmin(float x, float y)
{
    return (x < y) ? x : y;
}

// ************************************* current loop pi ******************************************
//
// ************************************************************************************************
float sat_lower_bound_q, sat_upper_bound_q, sat_minus_q;
float sat_lower_bound_d, sat_upper_bound_d, sat_minus_d;
float vd, vq, v_dq_norm;
float v_limit_pow2, vq_pow2, vd_pow2;
int vq_sign = 1;
static void cur_pi_run(qd_f_t *iqd_error, qd_f_t *BMF, float vlimit)
{
    cur_pi_regulator_.Ka_out.d = cur_pi_regulator_.Ka * iqd_error->d;
    cur_pi_regulator_.Ka_out.q = cur_pi_regulator_.Ka * iqd_error->q;

    // nominal Kb_out
    cur_pi_regulator_.Kb_out.d += cur_pi_regulator_.Ka_out.d * cur_pi_regulator_.Kb_const;
    cur_pi_regulator_.Kb_out.q += cur_pi_regulator_.Ka_out.q * cur_pi_regulator_.Kb_const;

    // pi saturation, only q axis
    sat_minus_q = cur_pi_regulator_.Ka_out.q + BMF->q;
    sat_lower_bound_q = my_fmin(0.f, -vlimit - sat_minus_q);
    sat_upper_bound_q = my_fmax(0.f, vlimit - sat_minus_q);

    sat_minus_d = cur_pi_regulator_.Ka_out.d + BMF->d;
    sat_lower_bound_d = my_fmin(0.f, -vlimit - sat_minus_d);
    sat_upper_bound_d = my_fmax(0.f, vlimit - sat_minus_d);

    cur_pi_regulator_.Kb_out.q = my_fmax(sat_lower_bound_q, my_fmin(cur_pi_regulator_.Kb_out.q, sat_upper_bound_q));
    cur_pi_regulator_.Kb_out.d = my_fmax(sat_lower_bound_d, my_fmin(cur_pi_regulator_.Kb_out.d, sat_upper_bound_d));

    // voltage
    vd = sat_minus_d + cur_pi_regulator_.Kb_out.d;
    vq = sat_minus_q + cur_pi_regulator_.Kb_out.q;
    if (vd > 0)
    {
        vd = (vd > vlimit) ? vlimit : vd;
    }
    else
    {
        vd = (vd < (-vlimit)) ? (-vlimit) : vd;
    }

    vq_sign = vq > 0 ? 1 : (-1);
    v_limit_pow2 = vlimit * vlimit;
    vd_pow2 = vd * vd;
    vq_pow2 = vq * vq;
    vd_pow2 = (vd_pow2 > v_limit_pow2) ? v_limit_pow2 : vd_pow2;
    // NOTE: the guard depends on axis-voltage
    if ((vd_pow2 + vq_pow2) > v_limit_pow2)
    {
        cur_pi_regulator_.Vqd_out.q = vq_sign * sqrtf(v_limit_pow2 - vd_pow2); // ensure the d-axis voltage.
    }
    else
    {
        cur_pi_regulator_.Vqd_out.q = vq;
    }
    cur_pi_regulator_.Vqd_out.d = vd;
}

static void cur_set_ka(float value)
{
    cur_pi_regulator_.Ka = value;
}
static void cur_set_kb(float value)
{
    cur_pi_regulator_.Kb = value;
}
static void cur_pi_reset(void)
{
    cur_pi_regulator_.Ka_out.d = cur_pi_regulator_.Ka_out.q = 0;
    cur_pi_regulator_.Kb_out.d = cur_pi_regulator_.Kb_out.q = 0;
    cur_pi_regulator_.Vqd_out.q = cur_pi_regulator_.Vqd_out.d = 0;
}

void Current_PI_Handler_Init(void)
{
    cur_pi_regulator_.pfct_cur_run = cur_pi_run;
    cur_pi_regulator_.pfct_cur_setka = cur_set_ka;
    cur_pi_regulator_.pfct_cur_setkb = cur_set_kb;
    cur_pi_regulator_.pfct_reset_cur = cur_pi_reset;

    cur_pi_regulator_.pfct_cur_setka(K_a);
    cur_pi_regulator_.pfct_cur_setkb(K_b);
    cur_pi_regulator_.Kb_const = K_b * T_PWM;
}

// ************************************* velocity loop pi ******************************************
//
// ************************************************************************************************
static void vel_pi_run(float fb_vel_value)
{
    vel_pi_regulator_.vel_last_ref = vel_pi_regulator_.vel_ref;
    vel_pi_regulator_.error = vel_pi_regulator_.vel_ref - fb_vel_value;
    vel_pi_regulator_.Ka_out = vel_pi_regulator_.error * vel_pi_regulator_.Ka;
    vel_pi_regulator_.Kb_out += (vel_pi_regulator_.Ka_out * vel_pi_regulator_.Kb_const);
    vel_pi_regulator_.iq_ref = vel_pi_regulator_.Ka_out + vel_pi_regulator_.Kb_out;
}
static void vel_set_ka(float value)
{
    vel_pi_regulator_.Ka = value;
}
static void vel_set_kb(float value)
{
    vel_pi_regulator_.Kb = value;
}
static void vel_pi_reset(void)
{
    vel_pi_regulator_.iq_ref = 0;
    vel_pi_regulator_.vel_last_ref = vel_pi_regulator_.vel_ref = 0;
    vel_pi_regulator_.Ka_out = vel_pi_regulator_.Kb_out = vel_pi_regulator_.error = 0;
}

void Vel_PI_Handler_Init(void)
{
    vel_pi_regulator_.Ka = K_av;
    vel_pi_regulator_.Kb = K_bv;
    vel_pi_regulator_.control_mode = Idle_Mode;
    vel_pi_regulator_.Kb_const = K_bv * T_vel;
    vel_pi_regulator_.pfct_vel_run = vel_pi_run;
    vel_pi_regulator_.pfct_vel_setka = vel_set_ka;
    vel_pi_regulator_.pfct_vel_setkb = vel_set_kb;
    vel_pi_regulator_.pfct_vel_reset = vel_pi_reset;
}

Cur_PI_Handler_t *get_cur_pi_handler(void)
{
    return &cur_pi_regulator_;
}

Vel_PI_Handler_t *get_vel_pi_handler(void)
{
    return &vel_pi_regulator_;
}