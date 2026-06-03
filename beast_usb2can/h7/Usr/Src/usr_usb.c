#include "usr_usb.h"

USB_PROTOCAL_TypeDef usb_protocal;

static void convert_usb_cmd(void) {}

static void convert_usb_data(void) {}

USB_PROTOCAL_TypeDef* get_usb_handle(void) {
  return &usb_protocal;
}

void usb_handle_init(void) {
  memset(&usb_protocal.usb_cmd_u, 0, sizeof(USB_CMD_U));
  memset(&usb_protocal.usb_data_u, 0, sizeof(USB_DATA_U));
  usb_protocal.convert_usb_data = convert_usb_data;
  usb_protocal.convert_usb_cmd  = convert_usb_cmd;
}