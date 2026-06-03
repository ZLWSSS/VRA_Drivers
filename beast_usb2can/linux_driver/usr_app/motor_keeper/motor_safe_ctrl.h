#ifndef MOTOR_SAFE_CTRL_H
#define MOTOR_SAFE_CTRL_H
#include <map>
#include <string>

// Constants for motor 8115
constexpr double battery_8115 = 36;
constexpr double GR_8115 = 9.75;
constexpr double Phi_8115 = 0.00597404;
constexpr double nominal_torque_8115 = 25; // Nm
constexpr double Npp_8115 = 21;
constexpr double Inverse_KT_OUT_8115 = 1.8488;
constexpr double KT = 1.0 / Inverse_KT_OUT_8115;
constexpr double Rs_8115 = 0.1625;
constexpr double Ls_8115 = 0.000105;
constexpr double K_Sim_8115 = 57;
constexpr double J_8115 = 0.0092;
constexpr double voltage_scale = 0.617476112898305; // Scale factor for voltage

constexpr double vel_deadband = 0.1; // rad/s

typedef enum {
    nothing = 0,
    motor_8115 = 1,
} Motor_Type;

typedef enum {
    nothing_acc = 0,
    negative_acc = 1,
    positive_acc = 2,
} Motor_State;

class Motor_Torque_Bound {
public:
    explicit Motor_Torque_Bound(Motor_Type motor_type, const double working_voltage = battery_8115) {
        if (motor_type == motor_8115) {
            motor_parameters_["working_voltage"] = working_voltage * voltage_scale;
            motor_parameters_["nominal_torque"] = nominal_torque_8115;
            motor_parameters_["gear_ratio"] = GR_8115;
            motor_parameters_["phi_m"] = Phi_8115;
            motor_parameters_["Npp"] = Npp_8115;
            motor_parameters_["inverse_KT_OUT"] = Inverse_KT_OUT_8115;
            motor_parameters_["Rs"] = Rs_8115;
            motor_parameters_["Ls"] = Ls_8115;
            motor_parameters_["K_Sim"] = K_Sim_8115;
            motor_parameters_["Inertial"] = J_8115;
            // motor_parameters_["alpha"] = Rs_8115 / Ls_8115 * K_Sim_8115 * Phi_8115;

            std::cout << "Parameters: " << motor_parameters_["alpha"] << ", " << motor_parameters_["beta"] << "\n";
        }
    }

    ~Motor_Torque_Bound() = default;

    double run_saturate_torque_reference(double torque, double qd, double ud, double uq, double tor_fb);

    double uq_left, sig_uq_left, sig_uq_left_last_{};
    double tor_max_{}, tor_min_{};

private:
    std::map<std::string, double> motor_parameters_;
    double original_torque_cmd_{}, original_torque_cmd_last_{};
    double saturate_torque_cmd_{}, saturate_torque_cmd_last_{};
    double torque_load_{};
    double state_vel_{}, state_vel_last_{};
    int b_acc_deacc_ = 0;


    Motor_State state_;
};

#endif
