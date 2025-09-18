#pragma once

#include "drv_usb_core.h"

void cdc_usr_recv_cpl(usb_dev* udev, int len, uint8_t* addr);
void cdc_usr_send_cpl(usb_dev* udev, int len, uint8_t* addr);
void cdc_usr_set_ready(usb_dev* udev);
