#include <cmath>
#include <iostream>

#include "driver/beast_usb2can.h"
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

    static int iter;
    static int start;
    while (true) {
        if (start && iter > 1000) {
            usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].t_ff_ = 3.0 * sinf(iter * 0.05);
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
    iox::runtime::PoshRuntime::initRuntime("Motor_CTRL_Node");
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
