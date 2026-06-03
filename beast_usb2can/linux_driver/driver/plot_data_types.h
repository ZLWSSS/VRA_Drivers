#ifndef PLOT_DATA_TYPES_H
#define PLOT_DATA_TYPES_H

typedef struct {
    double q_des, q;;
    double qd_des, qd;
    double tau_des, tau;
    double uq_esti, uq;
    double ud , acc ,temperature;
} Motor_Plot_t;

#endif