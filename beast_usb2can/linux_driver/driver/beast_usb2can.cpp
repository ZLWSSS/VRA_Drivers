#include "beast_usb2can.h"
#include "iostream"
#include <cstdlib>
#include <thread>
#include <cstdio>
#include <cstring>
#include <iomanip>

static uint32_t data_checksum(const uint32_t *data_to_check, uint32_t check_length) {
    uint32_t t = 0;
    for (int i = 0; i < check_length; i++) {
        t = t ^ data_to_check[i];
    }

    return t;
}

Beast_USB2CAN::Beast_USB2CAN(const uint16_t vendor_id, const uint16_t product_id, const uint8_t _motors_epin,
                             const uint8_t _motors_epout): motors_endpoint_in(_motors_epin),
                                                           motors_endpoint_out(_motors_epout),
                                                           usb_vendor_id(vendor_id),
                                                           usb_product_id(product_id),
                                                           LCM_Handle_usb_cmd_("udpm://239.255.76.67:7667?ttl=255"),
                                                           LCM_Handle_usb_data_("udpm://239.255.76.67:7667?ttl=255"),
                                                           motor_plot_publisher_({"PLOT", "MOTOR", "DATA"}) {
    usb_cmd_u = new USB_Cmd_U();
    usb_data_u = new USB_Data_U();
    control_cmd = new USB_Cmd_T();
    control_data = new USB_Data_T();

    time_last_out = std::chrono::steady_clock::now();
    time_now_out = std::chrono::steady_clock::now();

    time_last_in = std::chrono::steady_clock::now();
    time_now_in = std::chrono::steady_clock::now();
    libusb_init(&ctx);
    transfer_out_motors = libusb_alloc_transfer(0);
    transfer_in_motors = libusb_alloc_transfer(0);
    device_handle = libusb_open_device_with_vid_pid(ctx, usb_vendor_id, usb_product_id);
    if (device_handle == nullptr) {
        std::cerr << "Cannot find USB Device! PRODUCT ID: " << usb_product_id << " VENDOR ID: " << vendor_id <<
                std::endl;
        exit(EXIT_FAILURE);
    } else {
        std::cout << "Find USB Device OK!\n";
    }

    if (libusb_kernel_driver_active(device_handle, 0x00)) {
        int success = libusb_detach_kernel_driver(device_handle, 0x00);
        if (success != 0) {
            std::cerr << "Detach Driver Failed!" << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    int claim_interface = libusb_claim_interface(device_handle, 0x00);
    if (claim_interface != 0) {
        std::cerr << "Claim Driver Failed!" << std::endl;
        exit(EXIT_FAILURE);
    } else {
        std::cout << "Claim USB Device OK!\n";
    }

    // initiate files
    this->read_file_.open("motor_data.txt", std::ios::out | std::ios::trunc);
#ifdef WATCH_DATA
    thread_publisher_ = std::thread(&Beast_USB2CAN::thread_publisher_loop, this);
#endif
}
#ifdef WATCH_DATA
//WARNING choose your motor id here.
void Beast_USB2CAN::thread_publisher_loop() {
    while (true) {
        motor_plot_publisher_.loan().and_then([this](auto &sample) {
            std::shared_lock<std::shared_mutex> lk(this->usb_in_shared_mutex_);
            std::shared_lock<std::shared_mutex> lk_out(this->usb_out_shared_mutex_);
            sample->q_des = control_cmd->usb_chip_cmd_[0].cmd_pack[0].p_cmd_;
            sample->q = control_data->usb_chip_data_[0].data_pack[0].p_data_;
            sample->qd_des = control_cmd->usb_chip_cmd_[0].cmd_pack[0].v_cmd_;
            sample->qd = control_data->usb_chip_data_[0].data_pack[0].v_data_;
            sample->tau_des = control_cmd->usb_chip_cmd_[0].cmd_pack[0].t_ff_;
            sample->tau = control_data->usb_chip_data_[0].data_pack[0].t_data_;
            // sample->tau = 0;
            // sample->uq_esti = control_data->usb_chip_data_[0].data_pack[0].v_data_ * 9.75 * phi_m * K;
            sample->uq_esti = control_cmd->usb_chip_cmd_[0].cmd_pack[1].p_cmd_;
            sample->uq = control_data->usb_chip_data_[0].data_pack[0].uq_;
            sample->ud = control_data->usb_chip_data_[0].data_pack[0].ud_;
            sample->acc = control_cmd->usb_chip_cmd_[1].cmd_pack[0].t_ff_;
            sample->temperature = control_cmd->usb_chip_cmd_[1].cmd_pack[1].t_ff_;
            // sample->acc = control_data->usb_chip_data_[0].data_pack[0].acc_;
            // sample->temperature = control_data->usb_chip_data_[0].data_pack[0].coil_temp_;
            sample.publish();
            lk.unlock();
            lk_out.unlock();
        }).or_else([](auto &result) {
            std::cerr << "Unable to loan sample, error: " << result << std::endl;
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
#endif

Beast_USB2CAN::~Beast_USB2CAN() {
    this->read_file_.close();
    delete usb_cmd_u;
    delete usb_data_u;
    delete control_cmd;
    delete control_data;

    libusb_free_transfer(transfer_in_motors);
    libusb_free_transfer(transfer_out_motors);
    libusb_release_interface(device_handle, 0);
    libusb_close(device_handle);
    libusb_exit(ctx);
}


void Beast_USB2CAN::USB2CAN_SetBuffer(USB_Cmd_T *_control_cmd, USB_Data_T *_controller_data) {
    control_cmd = _control_cmd;
    control_data = _controller_data;
}

void Beast_USB2CAN::motor_epin_callback(struct libusb_transfer *_transfer) {
    if (_transfer->status != LIBUSB_TRANSFER_COMPLETED) {
        std::cout << "Motor Ep81 IN Error! Transfer again!\n";
    } else if (_transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        std::unique_lock<std::shared_mutex> lk(this->usb_in_shared_mutex_);
        this->Deal_Usb_In_Data();
        lk.unlock();
        if (time_spy_) {
            time_now_in = std::chrono::steady_clock::now();
            const std::chrono::duration<double, std::micro> time_used = std::chrono::duration_cast<std::chrono::duration
                <double,
                    std::micro> >(time_now_in - time_last_in);
            std::cout << "[Time Interval In]: " << time_used.count() << " us\n";
            time_last_in = time_now_in;
        }
    }
    libusb_submit_transfer(_transfer);
}

// std::cout << _transfer->status << "\n";


void Beast_USB2CAN::motor_epout_callback(struct libusb_transfer *_transfer) {
    if (_transfer->status != LIBUSB_TRANSFER_COMPLETED) {
        std::cout << "Motor Ep01 OUT Error! Transfer again!\n";
    } else if (_transfer->status == LIBUSB_TRANSFER_COMPLETED) {\
        // if (control_func_ != nullptr) { this->control_func_(); }
        std::shared_lock<std::shared_mutex> lk(this->usb_out_shared_mutex_);
        this->Deal_Usb_Out_Cmd();
        lk.unlock();
        if (time_spy_) {
            time_now_out = std::chrono::steady_clock::now();
            const std::chrono::duration<double, std::micro> time_used = std::chrono::duration_cast<std::chrono::duration
                <double,
                    std::micro> >(time_now_out - time_last_out);
            std::cout << "[Time Interval Out]: " << time_used.count() << " us\n";
            time_last_out = time_now_out;
        }
        libusb_submit_transfer(_transfer);
    }
}


void Beast_USB2CAN::Deal_Usb_In_Data() {
    const uint32_t t = data_checksum(reinterpret_cast<uint32_t *>(usb_data_u), usb_motors_in_check_length);
    volatile uint8_t leg_id = 0;
    volatile uint8_t data_index = 0;
    if (usb_data_u->usb_data_.checksum == t) {
        memcpy(control_data, usb_data_u, sizeof(USB_Data_T));
        if (write_) {
            read_file_ << std::setw(12) << std::setfill(' ') << usb_cmd_u->usb_cmd_.usb_chip_cmd_[0].cmd_pack[0].t_ff_
                    << " " <<
                    std::setw(12) << std::setfill(' ') << usb_data_u->usb_data_.usb_chip_data_[0].data_pack[0].p_data_
                    << " "
                    << std::setw(12) << std::setfill(' ') << usb_data_u->usb_data_.usb_chip_data_[0].data_pack[0].
                    v_data_ << " "
                    << std::setw(12) << std::setfill(' ') << usb_data_u->usb_data_.usb_chip_data_[0].data_pack[0].
                    t_data_ <<
                    " "
                    << std::setw(12) << std::setfill(' ') << usb_data_u->usb_data_.usb_chip_data_[0].data_pack[0].
                    uq_ << " "
                    << std::setw(12) << std::setfill(' ') << usb_data_u->usb_data_.usb_chip_data_[0].data_pack[0].
                    ud_ << " "
                    << std::setw(12) << std::setfill(' ') << usb_data_u->usb_data_.usb_chip_data_[0].data_pack[0].
                    acc_ << " "
                    << std::setw(12) << std::setfill(' ') << usb_data_u->usb_data_.usb_chip_data_[0].data_pack[0].
                    coil_temp_ << " "
                    << std::endl;
        }
    } else {
        std::cout << "usb data checksum error\n";
    }

    if (lcm_publish_) {
        memcpy(&usb_data_lcmt_, usb_data_u->usb_data_buff, sizeof(usb_data_lcmt_));
        LCM_Handle_usb_data_.publish("MOTOR DATA", &usb_data_lcmt_);
    }
}

void Beast_USB2CAN::Deal_Usb_Out_Cmd() {
    memcpy(usb_cmd_u, control_cmd, sizeof(USB_Cmd_T));
    usb_cmd_u->usb_cmd_.checksum = data_checksum(reinterpret_cast<uint32_t *>(usb_cmd_u), usb_motors_out_check_length);

    if (lcm_publish_) {
        memcpy(&usb_command_lcmt_, usb_cmd_u->usb_cmd_buff, sizeof(usb_command_lcmt_));
        // std::cout << usb_command_lcmt_.chip1_cmd[4] << "\n";
        LCM_Handle_usb_cmd_.publish("MOTOR COMMAND", &usb_command_lcmt_);
    }
}

void Beast_USB2CAN::start_transfer_ansy() {
    libusb_fill_interrupt_transfer(transfer_out_motors, device_handle, motors_endpoint_out,
                                   usb_cmd_u->usb_cmd_buff, usb_motors_out_length, usb_motors_out_cbf_wrapper, this, 0);
    libusb_fill_interrupt_transfer(transfer_in_motors, device_handle, motors_endpoint_in, usb_data_u->usb_data_buff,
                                   usb_motors_in_length, usb_motors_in_cbf_wrapper, this, 0);
    libusb_submit_transfer(transfer_in_motors);
    libusb_submit_transfer(transfer_out_motors);
    if ((!transfer_out_motors->status) & (!transfer_in_motors->status)) {
        std::cout << "[Good] All endpoints start transfering!\n";
    } else {
        std::cout << "[Bad] Some endpoints not work\n";
        exit(EXIT_FAILURE);
    }
}


void usb_motors_in_cbf_wrapper(struct libusb_transfer *_transfer) {
    auto *temp = reinterpret_cast<Beast_USB2CAN *>(_transfer->user_data);
    temp->motor_epin_callback(_transfer);
}

void usb_motors_out_cbf_wrapper(struct libusb_transfer *_transfer) {
    auto *temp = reinterpret_cast<Beast_USB2CAN *>(_transfer->user_data);
    temp->motor_epout_callback(_transfer);
}

void Beast_USB2CAN::start_transfer_sync() {
    this->Deal_Usb_Out_Cmd();
    int actual_length = 0;
    const bool success = libusb_interrupt_transfer(device_handle, motors_endpoint_out, usb_cmd_u->usb_cmd_buff,
                                                   usb_motors_out_length, &actual_length,
                                                   500);
    if (!success) {
        std::cout << "\n[Sych Transmit Success]\n";
    } else {
        std::cout << "\n[Transmit Error, Kill code and Try Again!] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
    }
}
