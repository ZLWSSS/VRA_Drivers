#include <cmath>
#include <iostream>
#include <driver/beast_usb2can.h>
#include <unistd.h>
#include "motor_safe_ctrl.h"
#include <sys/timerfd.h>

constexpr uint16_t beast_usb2can_vendor_id = 0x1111;
constexpr uint16_t beast_usb2can_product_id = 0x2222;
constexpr uint8_t motors_ep_in = 0x81;
constexpr uint8_t motors_ep_out = 0x01;
USB_Cmd_T *usb_cmd_from_controller;
USB_Data_T *usb_data_to_controller;
// 00000 00000 000 00
// cali 5bit | zero 5 bit | pos | vel | mit | disable | enable
[[noreturn]] void control_motors() {
    // construct the frequency:
    itimerspec timerSpec{};
    int timerfd = timerfd_create(CLOCK_MONOTONIC, 0);
    timerSpec.it_interval.tv_sec = 0;
    timerSpec.it_value.tv_sec = 0;
    timerSpec.it_value.tv_nsec = 1000000; // 1 ms
    timerSpec.it_interval.tv_nsec = 1000000; // 1 ms
    timerfd_settime(timerfd, 0, &timerSpec, nullptr);
    Motor_Torque_Bound motor_bound = Motor_Torque_Bound(motor_8115);

    static int iter;
    static int start;
    while (true) {
        if (start && iter > 1000) {
            // double fake_tff = 2.f * sinf(iter * 0.005);
            double fake_tff = -1.f;
            // usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].t_ff_ = static_cast<float>(
            // motor_bound.run_saturate_torque_reference(
            // fake_tff,
            // usb_data_to_controller->usb_chip_data_[0].data_pack[0].v_data_,
            // usb_data_to_controller->usb_chip_data_[0].data_pack[0].ud_,
            // usb_data_to_controller->usb_chip_data_[0].data_pack[0].uq_,
            // usb_data_to_controller->usb_chip_data_[0].data_pack[0].t_data_));
            usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].t_ff_ = fake_tff;
            usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[1].t_ff_ = static_cast<float>(
                motor_bound.run_saturate_torque_reference(
                    fake_tff,
                    usb_data_to_controller->usb_chip_data_[0].data_pack[0].v_data_,
                    usb_data_to_controller->usb_chip_data_[0].data_pack[0].ud_,
                    usb_data_to_controller->usb_chip_data_[0].data_pack[0].uq_,
                    usb_data_to_controller->usb_chip_data_[0].data_pack[0].t_data_));
            usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[1].p_cmd_ = motor_bound.sig_uq_left;
            usb_cmd_from_controller->usb_chip_cmd_[1].cmd_pack[0].t_ff_ = static_cast<float>(motor_bound.tor_max_);
            usb_cmd_from_controller->usb_chip_cmd_[1].cmd_pack[1].t_ff_ = static_cast<float>(motor_bound.tor_min_);
            // usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].t_ff_ = 0;
            // usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].v_cmd_ = 2.f;
            // usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].kd_ = 1.f;
        }
        iter++;
        if (iter < 100) {
            usb_cmd_from_controller->usb_chip_cmd_[0].chip_flag[0] = (0x01 << 1);
            usb_cmd_from_controller->usb_chip_cmd_[1].chip_flag[0] = (0x01 << 1);
            usb_cmd_from_controller->usb_chip_cmd_[2].chip_flag[0] = (0x01 << 1);
        } else {
            usb_cmd_from_controller->usb_chip_cmd_[0].chip_flag[0] = (0x01) | (0x01 << 2);
            usb_cmd_from_controller->usb_chip_cmd_[1].chip_flag[0] = (0x01) | (0x01 << 2);
            usb_cmd_from_controller->usb_chip_cmd_[2].chip_flag[0] = (0x01) | (0x01 << 2);
            start = 1;
        }
        unsigned long long missed = 0;
        const ssize_t m = read(timerfd, &missed, sizeof(missed));
        (void) m;
    }
}

[[noreturn]] int main() {
    usb_cmd_from_controller = new USB_Cmd_T();
    usb_data_to_controller = new USB_Data_T();
    int complete = 0;
    timeval timestru{};
    timestru.tv_sec = 0;
    timestru.tv_usec = 1000;
    iox::runtime::PoshRuntime::initRuntime("SAFE_CTRL_Node");
    Beast_USB2CAN spUSB2CAN(beast_usb2can_vendor_id, //
                            beast_usb2can_product_id,
                            motors_ep_in,
                            motors_ep_out);
    spUSB2CAN.USB2CAN_SetBuffer(usb_cmd_from_controller, usb_data_to_controller);
    // spUSB2CAN.register_ctrl_func(control_motors);
    spUSB2CAN.set_write_variable();
    // spUSB2CAN.set_time_spy();
    spUSB2CAN.set_lcm_publish();
    spUSB2CAN.start_transfer_ansy();
    auto thread_cmd_update = std::thread(control_motors);

    while (true) {
        libusb_handle_events_timeout_completed(spUSB2CAN.ctx, &timestru, &complete);
    }
}

/* suppose: the torque output always track the reference torque if no saturation.
 * the noise of the speed and acc are too large
 */

double Motor_Torque_Bound::run_saturate_torque_reference(double torque_ref, double qd, double ud, double uq, double tor_fb) {
    if (torque_ref > 0) {
        state_ = positive_acc;
    } else if (torque_ref < 0) {
        state_ = negative_acc;
    } else {
        return 0;
    }
    state_vel_last_ = qd;

    if (uq >= 0) {
        uq_left = sqrt(motor_parameters_["working_voltage"] * motor_parameters_["working_voltage"] - ud * ud) - uq;
        sig_uq_left = uq_left * (1 / (1 + exp(-0.5 * (uq_left - 3))));
        if (sig_uq_left < 2) {
            sig_uq_left = sig_uq_left * 0.4 + sig_uq_left_last_ *0.6;
        }
        sig_uq_left = sig_uq_left > motor_parameters_["working_voltage"] ? motor_parameters_["working_voltage"] : sig_uq_left;
        // sig_uq_left = std::abs(sig_uq_left) > 0.5 ? sig_uq_left : 0;
    } else {
        uq_left = -sqrt(motor_parameters_["working_voltage"] * motor_parameters_["working_voltage"] - ud * ud) - uq;
        sig_uq_left = uq_left * (1 - 1 / (1 + exp(-0.5 * (uq_left + 3))));
        // clip the uq_left to the working voltage
        if (sig_uq_left > -2) {
            sig_uq_left = sig_uq_left * 0.4 + sig_uq_left_last_ *0.6;
        }
        sig_uq_left = sig_uq_left < -motor_parameters_["working_voltage"] ? -motor_parameters_["working_voltage"] : sig_uq_left;
        // sig_uq_left = std::abs(sig_uq_left) > 0.5 ? sig_uq_left : 0;
    }
    sig_uq_left_last_ = sig_uq_left;
    std::cout << sig_uq_left << std::endl;

    if (uq > 0) {
        tor_max_ = sig_uq_left * J_8115 *1000 / (Phi_8115 * Npp_8115 * GR_8115) * 0.24;
        tor_max_ = tor_max_ > motor_parameters_["nominal_torque"] ? motor_parameters_["nominal_torque"] : tor_max_;
        tor_max_ = tor_max_ > 0.5 ?  tor_max_ : 0;
        tor_min_ = -motor_parameters_["nominal_torque"];
    }else if (uq < 0) {
        tor_min_ = sig_uq_left * J_8115 *1000 / (Phi_8115 * Npp_8115 * GR_8115) * 0.24;
        tor_min_ = tor_min_ < -motor_parameters_["nominal_torque"] ? -motor_parameters_["nominal_torque"] : tor_min_;
        tor_min_ = tor_min_ < -0.5 ?  tor_min_ : 0;
        tor_max_ = motor_parameters_["nominal_torque"];
    }
        // minus_emf = sig_uq_left / 0.001 - motor_parameters_["alpha"] * qd * GR_8115;
        // minus_emf = minus_emf > 0 ? 0 : minus_emf;
        // tor_min_ = motor_parameters_["Inertial"] / (motor_parameters_["beta"] * GR_8115) *
        // (minus_emf);
        // tor_min_ = (sig_uq_left + motor_parameters_["beta"] * saturate_torque_cmd_last_) / motor_parameters_["alpha"];
        // tor_max_ = motor_parameters_["nominal_torque"];
        // tor_min_ = -motor_parameters_["nominal_torque"];
        // tor_max_ = motor_parameters_["nominal_torque"];
    // case speed_keep:
    // torque_load_ = saturate_torque_cmd_last_;

    saturate_torque_cmd_ = std::clamp(torque_ref, tor_min_, tor_max_);
    // saturate_torque_cmd_ = std::abs(saturate_torque_cmd_) > 1 ? saturate_torque_cmd_ : 0;
    return saturate_torque_cmd_;
}
