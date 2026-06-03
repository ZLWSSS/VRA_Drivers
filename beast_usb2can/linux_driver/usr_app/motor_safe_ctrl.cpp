#include <cmath>
#include <iostream>

#include "driver/beast_usb2can.h"
#include <unistd.h>
#include <algorithm>
#include <random>

constexpr uint16_t beast_usb2can_vendor_id = 0x1111;
constexpr uint16_t beast_usb2can_product_id = 0x2222;
constexpr uint8_t motors_ep_in = 0x81;
constexpr uint8_t motors_ep_out = 0x01;
USB_Cmd_T *usb_cmd_from_controller;
USB_Data_T *usb_data_to_controller;

// motor parameters: 8115
constexpr double V_Scale = 0.617476112898305;
constexpr double Battery_voltage = 28 * V_Scale;
constexpr double GR = 9.75;
constexpr double Phi_m = 0.000692679;
constexpr double Npp = 21;
constexpr double Inverse_KT_OUT = 3.137493158;
constexpr double Rs = 0.071;
constexpr double Ls = 0.0000434;
constexpr double rs_ls = Rs / Ls;
constexpr double K_Sim = 66;
constexpr double J = 0.0007;
constexpr double T_MAX = 20; // nominal torque
constexpr double T_MIN = -20; // nominal torque
double t_cmd;
double peak = 10;
int peak_iter = 0;

void control_motors() {
    static int iter;
    static int start;

    if (start && iter > 1000) {
        // const double fake_ctrl_tff = 15.0 * sin(iter * 0.01);
        if ((iter - peak_iter) * 0.01 > 2 * M_PI) {
            peak_iter = iter;
            std::random_device seed;//硬件生成随机数种子
            std::ranlux48 engine(seed());//利用种子生成随机数引擎
            std::uniform_real_distribution<> distrib(5, 20);//设置随机数范围，并为均匀分布
            const double random = distrib(engine);
            peak = random;
            std::cout << peak << "sdsds\n";
        }

        const double fake_ctrl_tff = peak * sin(iter * 0.01);
        // constexpr double fake_ctrl_tff = 0;
        const double uq = usb_data_to_controller->usb_chip_data_[0].data_pack[0].uq_;
        const double ud = usb_data_to_controller->usb_chip_data_[0].data_pack[0].ud_;
        const double qd = usb_data_to_controller->usb_chip_data_[0].data_pack[0].v_data_;
        const double tau = usb_data_to_controller->usb_chip_data_[0].data_pack[0].t_data_;
        double uq_left = 0;
        double uq_left_sig = 0;
        if (fake_ctrl_tff >= 0) {
            uq_left = sqrt(Battery_voltage * Battery_voltage - ud * ud) - uq;
            uq_left_sig = uq_left * (1 / (1 + exp(-5 * (uq_left - 1))));
        } else {
            uq_left = -sqrt(Battery_voltage * Battery_voltage - ud * ud) - uq;
            uq_left_sig = uq_left * (1 - 1 / (1 + exp(-5 * (uq_left + 1))));
        }

        uq_left_sig = std::abs(uq_left_sig) > 0.1 ? uq_left_sig : 0;
        // const double rotor_acc = uq_left_sig / ((Npp + K_Sim) * Phi_m * 0.001f) - rs_ls * K_Sim * qd * GR / (Npp + K_Sim);
        // const double acc_aply = rotor_acc > 0 ? rotor_acc : 0;

        double t_cmd = uq_left_sig * J / (0.001 * Phi_m * (Npp + K_Sim)) / GR;
        // if (fake_ctrl_tff >= 0) {
            // usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].t_ff_ = static_cast<float>(std::clamp(fake_ctrl_tff, T_MIN, t_cmd));
        // } else {
            // usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].t_ff_ = static_cast<float>(std::clamp(fake_ctrl_tff, t_cmd, T_MAX));
        // }
        double sent_cmd = usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].t_ff_;
        usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].t_ff_ = fake_ctrl_tff;
        // next time uq (my)
        usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[2].p_cmd_ = static_cast<float>(uq + (Npp + K_Sim) * Phi_m * sent_cmd * GR / J * 0.001f);

        // next time uq (normal)
        usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[2].v_cmd_ = static_cast<float>(
            Rs * sent_cmd * Inverse_KT_OUT + qd * GR * (Npp + K_Sim) * Phi_m);
        usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[2].t_ff_ = static_cast<float>(-qd * GR * Npp * tau * Inverse_KT_OUT * Ls);
    }
    iter++;
    if (iter < 100) {
        usb_cmd_from_controller->usb_chip_cmd_[0].chip_flag[0] = (0x01 << 1);
    } else {
        usb_cmd_from_controller->usb_chip_cmd_[0].chip_flag[0] = (0x01) | (0x01 << 2);
        start = 1;

    }
    // usb_cmd_from_controller->usb_chip_cmd_[0].chip_flag[0] = 1 << 10;
}

[[noreturn]] int main() {
    usb_cmd_from_controller = new USB_Cmd_T();
    usb_data_to_controller = new USB_Data_T();
    int complete = 0;
    timeval timestru{};
    timestru.tv_sec = 0;
    timestru.tv_usec = 1000;

    Beast_USB2CAN spUSB2CAN(beast_usb2can_vendor_id, //
                            beast_usb2can_product_id,
                            motors_ep_in,
                            motors_ep_out);
    spUSB2CAN.USB2CAN_SetBuffer(usb_cmd_from_controller, usb_data_to_controller);
    spUSB2CAN.register_ctrl_func(control_motors);
    spUSB2CAN.set_write_variable();
    // spUSB2CAN.set_time_spy();
    spUSB2CAN.set_lcm_publish();
    spUSB2CAN.start_transfer_ansy();

    while (true) {
        libusb_handle_events_timeout_completed(spUSB2CAN.ctx, &timestru, &complete);
    }
}
