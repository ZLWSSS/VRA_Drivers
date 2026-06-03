#include <cmath>
#include <iostream>

#include "driver/beast_usb2can.h"
#include <unistd.h>

constexpr uint16_t beast_usb2can_vendor_id = 0x1111;
constexpr uint16_t beast_usb2can_product_id = 0x2222;
constexpr uint8_t motors_ep_in = 0x81;
constexpr uint8_t motors_ep_out = 0x01;
USB_Cmd_T *usb_cmd_from_controller;
USB_Data_T *usb_data_to_controller;
// 00000 00000 000 00
// cali 5bit | zero 5 bit | pos | vel | mit | disable | enable
void control_motors() {
    static int iter;
    static int start;
    if (start && iter > 1000) {
        usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].t_ff_ = 4.f;
        // usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].kd_ = 1.f;
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

constexpr double f_s = 20;
constexpr double f_e = 1000;
constexpr double sweep_time = 10;
constexpr double sample_rate = 1000;
constexpr double peak = 3; // torque peak
constexpr int data_length = sweep_time * sample_rate;
double cmds[data_length]{};
constexpr double dt = 1 / sample_rate;

// try linear:
// constexpr double delta_frequency = (f_e - f_s) / (sample_rate * sweep_time);
constexpr double delta_frequency = 1;
double frequency = f_s;
bool change = false;

void prepare_step_sweep() {
    int index_start = 0;
    int index_end = static_cast<int>(1.0 / frequency * sample_rate);
    std::cout << "index_end" << index_end << std::endl;
    double increment_frequency = 0;
    int index_point = 0;
    int delta_index = 0;
    while (1) {
        for (int i = index_start; i < index_end; i++) {
            if (!change)
                cmds[i] = peak;
            else
                cmds[i] = -peak;
            index_point++;
        }
        if (index_end == data_length) {
            break;
        }
        index_start = index_point;

        increment_frequency += delta_frequency;

        frequency = static_cast<int>(f_s + increment_frequency);
        delta_index = static_cast<int>(1.0 / frequency * sample_rate);
        if (delta_index == 0) {
            break;
        }
        index_end = index_start + delta_index;
        change = !change;
        index_end = index_end > data_length ? data_length : index_end;
        std::cout << delta_index << std::endl;
        std::cout << "frequency: " << frequency << std::endl;
        std::cout << "index_end" << index_end << std::endl;
        std::cout << "index_start" << index_start << std::endl;
    }
}

void step_sweep() {
    static int iter;
    static int start;
    static int cmd_index;
    if (start && iter > 1000) {
        if (cmd_index > data_length - 1) {
            usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].t_ff_ = 0;
        } else {
            usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].t_ff_ = cmds[cmd_index];
        }
        cmd_index++;
    }
    iter++;
    if (iter < 100) {
        usb_cmd_from_controller->usb_chip_cmd_[0].chip_flag[0] = 0x01 << 1;
    } else {
        usb_cmd_from_controller->usb_chip_cmd_[0].chip_flag[0] = (0x01) | (0x01 << 2);
        start = 1;
    }
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
    prepare_step_sweep();
    spUSB2CAN.register_ctrl_func(step_sweep);
    spUSB2CAN.set_write_variable();
    spUSB2CAN.set_lcm_publish();
    spUSB2CAN.start_transfer_ansy();

    while (true) {
        libusb_handle_events_timeout_completed(spUSB2CAN.ctx, &timestru, &complete);
    }
}
