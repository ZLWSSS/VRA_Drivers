//
// Created by lingwei on 3/26/24.
//

#ifndef PROJECT_RT_USB2CAN_H
#define PROJECT_RT_USB2CAN_H
#include <cstdint>
#include "libusb-1.0/libusb.h"
#include "lcm/lcm-cpp.hpp"
#include <mutex>
#include <fstream>
#include "beast_usb2can.h"
#include "lcm_types/cpp/usb_command_t.hpp"
#include "lcm_types/cpp/usb_data_t.hpp"
#include <shared_mutex>
#ifdef WATCH_DATA
#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "iox/signal_watcher.hpp"
#include "plot_data_types.h"
#endif

constexpr uint8_t NUMBER_CHIPS = 3;
constexpr int CTRL_MOTOR_NUM = 18;
constexpr int CHIP_CTRL_MOTOR_NUM = 6;
constexpr uint16_t usb_motors_in_length = 512;
constexpr uint16_t usb_motors_out_length = 376;
constexpr uint16_t usb_motors_in_check_length = usb_motors_in_length / 4 - 1;
constexpr uint16_t usb_motors_out_check_length = usb_motors_out_length / 4 - 1;
constexpr double phi_m = 0.00147095;
constexpr double K = 78;

typedef struct USB_CMD_Pack {
    float p_cmd_;
    float v_cmd_;
    float kp_;
    float kd_;
    float t_ff_;
} USB_CMD_PACK_T;

typedef struct USB_CHIP_CMD {
    USB_CMD_PACK_T cmd_pack[6];
    uint32_t chip_flag[1];
} USB_CHIP_CMD_T;

typedef struct USB_DATA_Pack {
    float p_data_;
    float v_data_;
    float t_data_;
    float uq_;
    float ud_;
    float acc_;
    float coil_temp_;
} USB_DATA_PACK_T;

typedef struct USB_CHIP_DATA {
    USB_DATA_PACK_T data_pack[6];
} USB_CHIP_DATA_T;

typedef struct USB_CMD {
    USB_CHIP_CMD_T usb_chip_cmd_[NUMBER_CHIPS];
    uint32_t checksum;
} USB_Cmd_T;

typedef union USB_CMD_UNION {
    USB_Cmd_T usb_cmd_;
    uint8_t usb_cmd_buff[usb_motors_out_length];
} USB_Cmd_U;

typedef struct USB_DATA {
    USB_CHIP_DATA_T usb_chip_data_[NUMBER_CHIPS];
    uint32_t motor_status_;
    uint32_t checksum;
} USB_Data_T;

typedef union USB_DATA_UNION {
    USB_Data_T usb_data_;
    uint8_t usb_data_buff[usb_motors_in_length];
} USB_Data_U;

class Beast_USB2CAN {
public:
    explicit Beast_USB2CAN(uint16_t vendor_id, uint16_t product_id, uint8_t _motors_epin, uint8_t _motors_epout);

    ~Beast_USB2CAN();

    // data union of this class is a temp buff, data checkok, memcpy to controll databuff.
    void USB2CAN_SetBuffer(USB_Cmd_T *_control_cmd, USB_Data_T *_controller_data);

    void start_transfer_ansy();

    void motor_epin_callback(struct libusb_transfer *_transfer);

    void motor_epout_callback(struct libusb_transfer *_transfer);


    libusb_context *ctx{};

    void set_write_variable() {
        write_ = true;
    }

    void set_time_spy() {
        time_spy_ = true;
    }

    void set_lcm_publish() {
        lcm_publish_ = true;
    }

    void register_ctrl_func(void (*func)(void)) {
        control_func_ = func;
    }

    void start_transfer_sync();

    std::shared_mutex usb_in_shared_mutex_;
    std::shared_mutex usb_out_shared_mutex_;

private:
    // for controller data protocals
    uint8_t motors_endpoint_in;
    uint8_t motors_endpoint_out;
    libusb_transfer *transfer_in_motors{};
    libusb_transfer *transfer_out_motors{};
    uint16_t usb_vendor_id;
    uint16_t usb_product_id;
    libusb_device_handle *device_handle{};

    USB_Data_U *usb_data_u{};
    USB_Cmd_U *usb_cmd_u{};

    USB_Cmd_T *control_cmd{};
    USB_Data_T *control_data{};

    void Deal_Usb_In_Data();

    void Deal_Usb_Out_Cmd();

    void (*control_func_)(void){};

    std::chrono::steady_clock::time_point time_last_in;
    std::chrono::steady_clock::time_point time_now_in;
    std::chrono::steady_clock::time_point time_last_out;
    std::chrono::steady_clock::time_point time_now_out;
    std::fstream read_file_;
    bool write_ = false;
    bool time_spy_ = false;
    bool lcm_publish_ = false;
    // LCM HANDLE
    lcm::LCM LCM_Handle_usb_cmd_;
    lcm::LCM LCM_Handle_usb_data_;
    usb_command_t usb_command_lcmt_{};
    usb_data_t usb_data_lcmt_{};


#ifdef WATCH_DATA
    iox::popo::Publisher<Motor_Plot_t> motor_plot_publisher_;
    std::thread thread_publisher_;

    [[noreturn]] void thread_publisher_loop();
#endif


    // for
};

void usb_motors_in_cbf_wrapper(struct libusb_transfer *_transfer);

void usb_motors_out_cbf_wrapper(struct libusb_transfer *_transfer);
#endif //PROJECT_RT_USB2CAN_H
