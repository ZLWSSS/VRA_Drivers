# VRA Drivers

Official hardware and software support code for the paper:

> **VRA: Grounding Discrete-Time Joint Acceleration in Voltage-Constrained Actuation**  
> Lingwei Zhang, Jiaming Wang, Tianlin Zhang, Zhitao Song, Xuanqi Zeng, Weipeng Xia, Zhongyu Li, and Yun-hui Liu  
> **Robotics: Science and Systems (RSS), 2026**

[[Paper]](https://arxiv.org/abs/2605.10696)

## Overview

This repository developed by the author contains the motor-driver firmware, USB-to-FDCAN communication tools, Linux-side control programs, and Simulink assets used in the development and hardware validation of **Voltage-Realizable Acceleration (VRA)**. Thanks to the support of HKCLR and CUHK Legged Robot Lab, we provide the Schematic Diagrams and Gerbers of the boards.   

VRA is a joint-level acceleration interface for robots driven by voltage-constrained electric actuators. Instead of treating kinematically admissible accelerations as automatically executable, VRA explicitly accounts for actuator voltage realizability and restricts commanded accelerations to those that can be physically produced by the motor-drive system.

The code in this repository supports:

- Embedded motor control for the experimental actuator platform.
- USB-to-FDCAN communication between the host computer and embedded controllers.
- Linux-side motor command and test utilities.
- PMSM/FOC Simulink models and experiment-related analysis scripts.

## Repository Structure

```text
VRA_Drivers/
├── Beast_Driver/               # Embedded motor-driver firmware
│   ├── Core/
│   ├── Drivers/
│   ├── G4_Support/
│   └── usr/                    # Motor control, CAN, encoder, SVPWM, FSM, and configuration code
│
├── beast_usb2can/              # USB-to-CAN communication implementation
│   ├── g4/                     # STM32G4-side firmware
│   ├── h7/                     # STM32H7 USB-to-CAN firmware
│   └── linux_driver/           # Linux host-side driver and motor-control tools
│
├── VRA_Simulink/               # Simulink models, experimental data, and MATLAB scripts
│   ├── my_foc_TI.slx
│   ├── plot_data.m
│   ├── plot_datas.m
│   └── scan_the_damping_factor.m
│
└── LICENSE
````

## Components

### 1. `Beast_Driver`

`Beast_Driver` contains the embedded firmware for the motor-drive platform used in the VRA experiments.

The user-level implementation in `Beast_Driver/usr/` includes:

* CAN communication handling.
* Motor and actuator configuration.
* Encoder processing.
* PI regulators.
* Space-vector PWM generation.
* Motor-state monitoring and error handling.
* Calibration and finite-state-machine logic.

Motor-related parameters and controller configuration are provided in:

```text
Beast_Driver/usr/motor_config.h
```

Before flashing the firmware, check and configure the motor type and CAN ID in `motor_config.h`.

The current source tree includes parameter definitions for multiple motor configurations, including:

```text
MOTOR_8110
MOTOR_8115
MOTOR_6210
```

### 2. `beast_usb2can`

`beast_usb2can` provides communication support between the host computer and the motor controller.

It includes:

* `g4/`: STM32G4 firmware for CAN-related communication.
* `h7/`: STM32H7 firmware implementing the USB-to-CAN interface.
* `linux_driver/`: Linux-side C++ programs for communicating with the embedded motor controllers.

The Linux-side source tree includes:

```text
linux_driver/
├── driver/
│   ├── beast_usb2can.cpp
│   └── beast_usb2can.h
└── usr_app/
    ├── motor_ctrl.cpp
    ├── motor_safe_ctrl.cpp
    ├── motor_step_sweep.cpp
    ├── motor_tools.cpp
    └── lcm_reader.cpp
```

### 3. `VRA_Simulink`

`VRA_Simulink` contains PMSM/FOC simulation assets and experiment-related MATLAB scripts.

Key files include:

```text
my_foc_TI.slx                  # Simulink motor-control model
scan_the_damping_factor.m      # Parameter scanning script
```

These assets are provided to support analysis of actuator-level behavior and voltage-constrained motor execution associated with VRA.

## Getting Started

### Clone the Repository

```bash
git clone https://github.com/ZLWSSS/VRA_Drivers.git
cd VRA_Drivers
```

## Embedded Firmware

### Motor Driver Firmware

The motor-driver project is located in:

```text
Beast_Driver/
```

Project files are provided for STM32-based development workflows, including:

```text
beast_motor_driver.ioc
beast_motor_driver.emProject
```

A typical workflow is:

1. Open the project using the corresponding STM32/embedded development environment.
2. Configure the motor type and CAN ID in `Beast_Driver/usr/motor_config.h`.
3. Build and flash the firmware to the motor-control board.
4. Verify encoder, communication, and motor configuration before enabling powered motion.

### USB-to-CAN Firmware

The USB-to-CAN implementation is located in:

```text
beast_usb2can/
```

Select the firmware directory corresponding to the target microcontroller:

```text
beast_usb2can/g4/
beast_usb2can/h7/
```

The H7 project includes the USB device middleware used for host communication.

## Linux Host-Side Tools

The Linux-side driver is located in:

```bash
cd beast_usb2can/linux_driver
```

### Dependencies

The current CMake configuration links against:

* `libusb-1.0`
* `pthread`
* `lcm`
* `iceoryx_posh`

Ensure these dependencies are installed and discoverable by CMake before building.

### Build

```bash
mkdir -p build
cd build
cmake ..
make -j
```

The current `CMakeLists.txt` defines the following executables:

```text
MOTOR_CTRL
```

These programs provide host-side utilities for motor communication, motor-control experiments, safety-oriented control, data reading, and test procedures.

## Simulink and MATLAB Assets

The Simulink model and MATLAB scripts are located in:

Open the primary model in MATLAB/Simulink:

```matlab
open_system('my_foc_TI.slx')
```

Additional analysis scripts are provided for visualizing recorded signals and inspecting damping-related behavior.

The authors are not responsible for hardware damage or unsafe operation resulting from incorrect configuration or misuse of the code.

## Citation

When using this repository in academic work, please cite:

```bibtex
@article{zhang2026vra,
  title   = {{VRA}: Grounding Discrete-Time Joint Acceleration in Voltage-Constrained Actuation},
  author  = {Zhang, Lingwei and Wang, Jiaming and Zhang, Tianlin and Song, Zhitao and Zeng, Xuanqi and Xia, Weipeng and Li, Zhongyu and Liu, Yun-hui},
  journal = {Robotics: Science and Systems},
  year    = {2026}
}
```
