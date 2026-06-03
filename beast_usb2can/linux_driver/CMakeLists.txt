cmake_minimum_required(VERSION 3.5)
project(USB2CAN_Linux)

set(CMAKE_CXX_STANDARD 14)

set(INC_DIR /usr/local/include)
set(LINK_DIR /usr/local/lib)

include_directories(${PROJECT_SOURCE_DIR}/driver)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
include_directories(${INC_DIR})
link_directories(${LINK_DIR})

if (WATCH_DATA_FLG)
    message(WARNING "Build include iceoryx")
    add_compile_definitions(WATCH_DATA)
endif ()


include(GNUInstallDirs)
#include_directories("/home/hb/legged/iceoryx/")
find_package(iceoryx_platform REQUIRED)
find_package(iceoryx_posh CONFIG REQUIRED)
find_package(iceoryx_hoofs CONFIG REQUIRED)

add_executable(MOTOR_STEP_SWEEP usr_app/motor_step_sweep.cpp driver/beast_usb2can.cpp)
target_link_libraries(MOTOR_STEP_SWEEP libusb-1.0.so pthread lcm iceoryx_posh::iceoryx_posh)

add_executable(MOTOR_CTRL usr_app/motor_ctrl.cpp driver/beast_usb2can.cpp)
target_link_libraries(MOTOR_CTRL libusb-1.0.so pthread lcm iceoryx_posh::iceoryx_posh)

add_executable(MOTOR_TOOL usr_app/motor_tools.cpp driver/beast_usb2can.cpp)
target_link_libraries(MOTOR_TOOL libusb-1.0.so pthread lcm iceoryx_posh::iceoryx_posh)

add_executable(MOTOR_SC usr_app/motor_safe_ctrl.cpp driver/beast_usb2can.cpp)
target_link_libraries(MOTOR_SC libusb-1.0.so pthread lcm iceoryx_posh::iceoryx_posh)

add_executable(LCM_READER usr_app/lcm_reader.cpp)
target_link_libraries(LCM_READER lcm)

add_executable(MOTOR_KEEPER usr_app/motor_keeper/motor_safe_ctrl.cpp driver/beast_usb2can.cpp)
target_link_libraries(MOTOR_KEEPER libusb-1.0.so pthread lcm iceoryx_posh::iceoryx_posh)

