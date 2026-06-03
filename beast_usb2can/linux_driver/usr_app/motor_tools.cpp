#include <cmath>
#include <cstring>
#include "beast_usb2can.h"
#include <unistd.h>
#include <iostream>

constexpr uint16_t beast_usb2can_vendor_id = 0x1111;
constexpr uint16_t beast_usb2can_product_id = 0x2222;
constexpr uint8_t motors_ep_in = 0x81;
constexpr uint8_t motors_ep_out = 0x01;
USB_Cmd_T *usb_cmd_from_controller;
USB_Data_T *usb_data_to_controller;
#define CALI_SHIFT 10
#define ZERO_SHIFT 5

// 000       000         0             000         000
// cali 3 bits | zero 3 bits  | normal_ctrl | 3 disable | 3 enable.

[[noreturn]] int main() {
    usb_cmd_from_controller = new USB_Cmd_T();
    usb_data_to_controller = new USB_Data_T();

    Beast_USB2CAN spUSB2CAN(beast_usb2can_vendor_id, //
                            beast_usb2can_product_id,
                            motors_ep_in,
                            motors_ep_out);
    spUSB2CAN.USB2CAN_SetBuffer(usb_cmd_from_controller, usb_data_to_controller);
    spUSB2CAN.set_write_variable();

    char input[20]{};
    char cmd_type{};
    int motor_index = 0;
    while (true) {
        std::cout
                << "\nPlease Choose Cmd:\n CMD Format: [[C] for calibration; [Z] for zero; [G] for read and cmd motors][space][motorindex(0~17)]\n";
        std::cin.getline(input, 20);
        cmd_type = input[0];
        if (islower(cmd_type)) {
            cmd_type = toupper(cmd_type);
        }

        switch (cmd_type) {
            case 'C':
                if (std::strlen(input) == 4) {
                    motor_index = int(input[2] - '0') * 10 + int(input[3] - '0');
                } else if (std::strlen(input) == 3) {
                    motor_index = int(input[2] - '0');
                }
            std::cout << "\033[33m" << "[Caliration] Motor Index " << motor_index << "\033[0m";
            if (motor_index < 0 || motor_index > (CTRL_MOTOR_NUM - 1)) {
                std::cout << "\033[1m\033[31m" << "[ERROR] Index out of Range\n" << "\033[0m" << std::endl;
            } else {
                usb_cmd_from_controller->usb_chip_cmd_[0].chip_flag[0] = 0;
                usb_cmd_from_controller->usb_chip_cmd_[1].chip_flag[0] = 0;
                usb_cmd_from_controller->usb_chip_cmd_[2].chip_flag[0] = 0;
                const int chip_id = motor_index / CHIP_CTRL_MOTOR_NUM;
                const int motor_number = motor_index % CHIP_CTRL_MOTOR_NUM + 1;
                usb_cmd_from_controller->usb_chip_cmd_[chip_id].chip_flag[0] = static_cast<uint32_t>(motor_number) << CALI_SHIFT;
                spUSB2CAN.start_transfer_sync();
                std::cout << "\033[33m" << ", Chip ID " << chip_id << ", motor_number " << motor_number << "\033[0m\n";
            }
            break;
            case 'Z':
                if (std::strlen(input) == 4) {
                    motor_index = int(input[2] - '0') * 10 + int(input[3] - '0');
                } else if (std::strlen(input) == 3) {
                    motor_index = int(input[2] - '0');
                }
            std::cout << "\033[33m" << "[Zero] Motor Index " << motor_index << "\033[0m";
            if (motor_index < 0 || motor_index > (CTRL_MOTOR_NUM - 1)) {
                std::cout << "\033[1m\033[31m" << "[ERROR] Index out of Range\n" << "\033[0m" << std::endl;
            } else {
                usb_cmd_from_controller->usb_chip_cmd_[0].chip_flag[0] = 0;
                usb_cmd_from_controller->usb_chip_cmd_[1].chip_flag[0] = 0;
                usb_cmd_from_controller->usb_chip_cmd_[2].chip_flag[0] = 0;
                const int chip_id = motor_index / 6;
                const int motor_number = motor_index % 6 + 1;
                usb_cmd_from_controller->usb_chip_cmd_[chip_id].chip_flag[0] = static_cast<uint32_t>(motor_number) << ZERO_SHIFT;
                spUSB2CAN.start_transfer_sync();
                std::cout << "\033[33m" << ", Chip ID " << chip_id << ", motor_number " << motor_number << "\033[0m\n";
            }
            break;
            case 'A':
                for (int i = 0; i < CTRL_MOTOR_NUM; i++) {
                    const int chip_id = i / CHIP_CTRL_MOTOR_NUM;
                    const int motor_number = i % CHIP_CTRL_MOTOR_NUM + 1;
                    usb_cmd_from_controller->usb_chip_cmd_[chip_id].chip_flag[0] = static_cast<uint32_t>(motor_number) << ZERO_SHIFT;
                    spUSB2CAN.start_transfer_sync();
                    std::cout << "Motor Index: " << i << std::endl;
                    usleep(1000000);
                }
            break;
            default:
                std::cout << "\033[1m\033[31m" << "Cmd Not Recognized!\n" << "\033[0m" << std::endl;
            break;
        }
    }
}

