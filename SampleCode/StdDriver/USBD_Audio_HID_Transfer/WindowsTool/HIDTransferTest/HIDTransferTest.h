#pragma once

#include "resource.h"

#define USB_VID         0x0416  /* Vendor ID */
#define USB_PID         0xB00A  /* Product ID */

#define HID_CMD_SIGNATURE   0x43444948

/* HID Transfer Commands */
#define HID_CMD_NONE     0x00
#define HID_CMD_ERASE    0x71
#define HID_CMD_READ     0xD2
#define HID_CMD_WRITE    0xC3
#define HID_CMD_TEST     0xB4


#define PAGE_SIZE       256
#define SECTOR_SIZE     4096
#define HID_PACKET_SIZE 32


#define USB_TIME_OUT    100

